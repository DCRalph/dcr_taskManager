#include "dcr_TaskManager.h"

#include <algorithm>

#include <esp_err.h>
#include <esp_freertos_hooks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if defined(CONFIG_ESP_TASK_WDT) && CONFIG_ESP_TASK_WDT
#include <esp32-hal.h>
#include <esp_task_wdt.h>
#endif

#include <dcr_taskManager/FreeRtosRaii.h>
#include <dcr_taskManager/MutexRegistry.h>
#include <dcr_Logger.h>

#undef LOG_TAG
#define LOG_TAG "TASKMGR"

namespace
{
  struct TaskRecord
  {
    String name;
    TaskHandle_t handle = nullptr;
    BaseType_t core = tskNO_AFFINITY;
    UBaseType_t priority = 0;
    uint32_t stackSize = 0;
    uint32_t lastHeartbeatMs = 0;
  };

  std::vector<TaskRecord> gTaskRecords;

  FreeRtosRaii::Mutex &taskRegistryMutex()
  {
    static FreeRtosRaii::Mutex m;
    return m;
  }

  // Serializes refreshIdleEstimates() so concurrent const getters don't both
  // run the read-modify-write on the idle sample state.
  FreeRtosRaii::Mutex &idleStatsMutex()
  {
    static FreeRtosRaii::Mutex m;
    return m;
  }

  // Idle hooks run from FreeRTOS built-in idle tasks. Returning true limits callbacks to once per tick.
  volatile uint32_t gIdleTickSamples[portNUM_PROCESSORS] = {0};

  uint32_t gPrevIdleSamples[portNUM_PROCESSORS] = {0};
  uint32_t gPrevSampleMs = 0;
  float gIdlePercent[portNUM_PROCESSORS] = {0.f};
  bool gIdleMonitorStarted = false;

  constexpr uint32_t kIdleRefreshMs = 250;

  void refreshIdleEstimates()
  {
    const uint32_t ms = millis();
    if (ms - gPrevSampleMs < kIdleRefreshMs)
      return;

    // getIdlePercent()/getCpuUsagePercent() are const and called from several
    // tasks, so two callers can reach here in the same window. Without
    // serialization each runs the read-modify-write on gPrevIdleSamples /
    // gPrevSampleMs and the second sees a ~zero delta, reporting a bogus CPU%.
    // Let one task own the window; the others keep the last values (aligned
    // reads are atomic on Xtensa, so a stale read is harmless).
    auto lock = FreeRtosRaii::tryLock(idleStatsMutex(), 0);
    if (!lock)
      return;

    const uint32_t dt = ms - gPrevSampleMs;
    if (dt < kIdleRefreshMs)
      return;

    const float hz = static_cast<float>(configTICK_RATE_HZ);
    for (BaseType_t c = 0; c < portNUM_PROCESSORS; ++c)
    {
      const uint32_t cur = gIdleTickSamples[c];
      const uint32_t ds = cur - gPrevIdleSamples[c];
      gPrevIdleSamples[c] = cur;

      float pct = 0.f;
      if (hz > 0.f && dt > 0)
      {
        const float rate = static_cast<float>(ds) * 1000.f / static_cast<float>(dt);
        pct = 100.f * rate / hz;
        if (pct > 100.f)
          pct = 100.f;
      }
      gIdlePercent[c] = pct;
    }
    gPrevSampleMs = ms;
  }

  extern "C" bool taskMgrIdleHook0(void)
  {
    gIdleTickSamples[0]++;
    return true;
  }

#if portNUM_PROCESSORS > 1
  extern "C" bool taskMgrIdleHook1(void)
  {
    gIdleTickSamples[1]++;
    return true;
  }
#endif
}

void TaskManager::begin()
{
  if (gIdleMonitorStarted)
    return;

  esp_err_t reg0 = esp_register_freertos_idle_hook_for_cpu(taskMgrIdleHook0, 0);
  if (reg0 != ESP_OK)
    debugW("idle hook core0 failed err=%d", static_cast<int>(reg0));

#if portNUM_PROCESSORS > 1
  esp_err_t reg1 = esp_register_freertos_idle_hook_for_cpu(taskMgrIdleHook1, 1);
  if (reg1 != ESP_OK)
    debugW("idle hook core1 failed err=%d", static_cast<int>(reg1));
#endif

#if defined(CONFIG_ESP_TASK_WDT) && CONFIG_ESP_TASK_WDT
  disableLoopWDT();
  disableCore0WDT();
#if portNUM_PROCESSORS > 1
  disableCore1WDT();
#endif
  esp_err_t twdtEnd = esp_task_wdt_deinit();
  if (twdtEnd != ESP_OK && twdtEnd != ESP_ERR_NOT_FOUND)
    debugW("esp_task_wdt_deinit err=%d", static_cast<int>(twdtEnd));
#endif

  gPrevSampleMs = millis();
  gIdleMonitorStarted = true;
  RtosUtils::registerMutex(taskRegistryMutex(), "taskRegistry");
  debugI("Idle sampling via built-in idle tasks (hooks @ tick rate)");
}

bool TaskManager::createTaskPinnedToCore(TaskFunction_t taskFn, const char *name,
                                         uint32_t stackSize, void *param,
                                         UBaseType_t priority, TaskHandle_t *outHandle,
                                         BaseType_t core)
{
  TaskHandle_t createdHandle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(taskFn, name, stackSize, param, priority,
                                              &createdHandle, core);
  if (result != pdPASS || createdHandle == nullptr)
    return false;

  if (outHandle != nullptr)
    *outHandle = createdHandle;

  registerTask(createdHandle, name, stackSize, priority, core);
  return true;
}

void TaskManager::registerTask(TaskHandle_t handle, const char *name, uint32_t stackSize,
                               UBaseType_t priority, BaseType_t core)
{
  if (handle == nullptr)
    return;

  std::lock_guard<FreeRtosRaii::Mutex> lock(taskRegistryMutex());
  auto it = std::find_if(gTaskRecords.begin(), gTaskRecords.end(),
                         [handle](const TaskRecord &record)
                         {
                           return record.handle == handle;
                         });
  if (it == gTaskRecords.end())
  {
    gTaskRecords.push_back(TaskRecord{
        name != nullptr ? String(name) : String(),
        handle,
        core,
        priority,
        stackSize,
        millis(),
    });
  }
  else
  {
    it->name = name != nullptr ? String(name) : String();
    it->core = core;
    it->priority = priority;
    it->stackSize = stackSize;
  }
}

void TaskManager::noteHeartbeat(TaskHandle_t handle)
{
  TaskHandle_t target = handle != nullptr ? handle : xTaskGetCurrentTaskHandle();
  if (target == nullptr)
    return;

  std::lock_guard<FreeRtosRaii::Mutex> lock(taskRegistryMutex());
  auto it = std::find_if(gTaskRecords.begin(), gTaskRecords.end(),
                         [target](const TaskRecord &record)
                         {
                           return record.handle == target;
                         });
  if (it != gTaskRecords.end())
    it->lastHeartbeatMs = millis();
}

void TaskManager::unregisterTask(TaskHandle_t handle)
{
  TaskHandle_t target = handle != nullptr ? handle : xTaskGetCurrentTaskHandle();
  if (target == nullptr)
    return;

  std::lock_guard<FreeRtosRaii::Mutex> lock(taskRegistryMutex());
  gTaskRecords.erase(
      std::remove_if(gTaskRecords.begin(), gTaskRecords.end(),
                     [target](const TaskRecord &record)
                     { return record.handle == target; }),
      gTaskRecords.end());
}

float TaskManager::getIdlePercent(int core) const
{
  refreshIdleEstimates();
  if (core >= 0 && core < portNUM_PROCESSORS)
    return gIdlePercent[core];
  return 0.f;
}

float TaskManager::getCpuUsagePercent(int core) const
{
  if (core >= 0 && core < portNUM_PROCESSORS)
    return 100.f - getIdlePercent(core);

  refreshIdleEstimates();
  float sum = 0.f;
  for (BaseType_t c = 0; c < portNUM_PROCESSORS; ++c)
    sum += 100.f - gIdlePercent[c];
  return sum / static_cast<float>(portNUM_PROCESSORS);
}

std::vector<TaskSnapshot> TaskManager::snapshotTasks() const
{
  std::lock_guard<FreeRtosRaii::Mutex> lock(taskRegistryMutex());
  std::vector<TaskSnapshot> snapshots;
  snapshots.reserve(gTaskRecords.size());

  for (const TaskRecord &record : gTaskRecords)
  {
    if (record.handle == nullptr)
      continue;

    // A task that self-deleted without unregistering leaves a stale handle
    // here; querying a deleted or recycled TCB returns garbage or faults. Skip
    // any handle the kernel no longer reports as live.
    const eTaskState state = eTaskGetState(record.handle);
    if (state == eDeleted || state == eInvalid)
      continue;

    TaskSnapshot snapshot;
    snapshot.name = record.name;
    snapshot.handle = record.handle;
    snapshot.core = record.core;
    snapshot.priority = record.priority;
    snapshot.stackSize = record.stackSize;
    snapshot.lastHeartbeatMs = record.lastHeartbeatMs;
    snapshot.stackHighWaterMarkWords = uxTaskGetStackHighWaterMark(record.handle);
    snapshots.push_back(snapshot);
  }

  return snapshots;
}

TaskManager taskManager;
