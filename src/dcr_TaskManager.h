#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

struct TaskSnapshot
{
  String name;
  TaskHandle_t handle = nullptr;
  BaseType_t core = tskNO_AFFINITY;
  UBaseType_t priority = 0;
  uint32_t stackSize = 0;
  uint32_t stackHighWaterMarkWords = 0;
  uint32_t lastHeartbeatMs = 0;
};

class TaskManager
{
public:
  void begin();

  bool createTaskPinnedToCore(TaskFunction_t taskFn, const char *name,
                              uint32_t stackSize, void *param,
                              UBaseType_t priority, TaskHandle_t *outHandle,
                              BaseType_t core);
  void registerTask(TaskHandle_t handle, const char *name, uint32_t stackSize,
                    UBaseType_t priority, BaseType_t core);
  void noteHeartbeat(TaskHandle_t handle = nullptr);
  // Remove a task's registry record. Self-deleting tasks MUST call this before
  // vTaskDelete() — otherwise their handle dangles in the registry and
  // snapshotTasks() later queries a freed/recycled TCB. Defaults to the caller.
  void unregisterTask(TaskHandle_t handle = nullptr);

  float getIdlePercent(int core) const;
  float getCpuUsagePercent(int core = -1) const;
  std::vector<TaskSnapshot> snapshotTasks() const;
};

extern TaskManager taskManager;
