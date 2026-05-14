#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace FreeRtosRaii
{
  struct DeferredCreateTag
  {
  };

  inline constexpr DeferredCreateTag DeferredCreate{};

  namespace detail
  {
    inline uint32_t ticksToMillis(TickType_t ticks)
    {
      return static_cast<uint32_t>(pdTICKS_TO_MS(ticks));
    }

    inline uint32_t nowMs()
    {
      return ticksToMillis(xTaskGetTickCount());
    }

    inline TickType_t durationToTicks(const std::chrono::nanoseconds &duration)
    {
      if (duration <= std::chrono::nanoseconds::zero())
        return 0;

      const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
      if (millis.count() >= static_cast<int64_t>(std::numeric_limits<uint32_t>::max()))
        return portMAX_DELAY;

      const uint32_t ms = static_cast<uint32_t>(millis.count());
      TickType_t ticks = pdMS_TO_TICKS(ms);
      if (ticks == 0 && ms > 0)
        ticks = 1;
      return ticks;
    }

    struct OwnerState
    {
      TaskHandle_t task = nullptr;
      const char *taskName = nullptr;
      const char *context = nullptr;
      uint32_t sinceMs = 0;
      uint16_t depth = 0;
    };

    template <typename Derived>
    class OwnerTrackedMutexBase
    {
    public:
      struct OwnerInfo
      {
        TaskHandle_t task = nullptr;
        const char *taskName = nullptr;
        const char *context = nullptr;
        uint32_t sinceMs = 0;
        uint16_t depth = 0;
      };

      OwnerInfo ownerInfo() const noexcept
      {
        return {
            _owner.task,
            _owner.taskName,
            _owner.context,
            _owner.sinceMs,
            _owner.depth};
      }

      TaskHandle_t ownerTask() const noexcept { return _owner.task; }
      const char *ownerTaskName() const noexcept { return _owner.taskName; }
      const char *ownerContext() const noexcept { return _owner.context; }
      uint32_t ownerSinceMs() const noexcept { return _owner.sinceMs; }
      uint16_t ownerDepth() const noexcept { return _owner.depth; }

      uint32_t ownerHeldMs(uint32_t nowMs = detail::nowMs()) const noexcept
      {
        if (_owner.sinceMs == 0 || nowMs < _owner.sinceMs)
          return 0;
        return nowMs - _owner.sinceMs;
      }

      bool take(TickType_t timeout = portMAX_DELAY, const char *context = nullptr)
      {
        Derived &self = static_cast<Derived &>(*this);
        if (!self.ensureCreated())
          return false;
        if (!self.takeImpl(timeout))
          return false;

        noteAcquired(context);
        return true;
      }

      void give()
      {
        Derived &self = static_cast<Derived &>(*this);
        configASSERT(self.valid());
        noteReleased();
        self.giveImpl();
      }

      void lock()
      {
        configASSERT(take(portMAX_DELAY));
      }

      bool try_lock()
      {
        return take(0);
      }

      template <class Rep, class Period>
      bool try_lock_for(const std::chrono::duration<Rep, Period> &timeout)
      {
        return take(detail::durationToTicks(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)));
      }

      void unlock()
      {
        give();
      }

    protected:
      void noteAcquired(const char *context)
      {
        TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
        const char *name = currentTask ? pcTaskGetName(currentTask) : nullptr;
        if (_owner.task == currentTask && currentTask != nullptr)
        {
          ++_owner.depth;
          if (context)
            _owner.context = context;
          return;
        }

        _owner.task = currentTask;
        _owner.taskName = name;
        _owner.context = context ? context : "unspecified";
        _owner.sinceMs = detail::nowMs();
        _owner.depth = 1;
      }

      void noteReleased()
      {
        TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
        if (_owner.task == currentTask && _owner.depth > 0)
        {
          --_owner.depth;
          if (_owner.depth == 0)
            clearOwner();
          return;
        }

        clearOwner();
      }

      void clearOwner()
      {
        _owner = {};
      }

    private:
      OwnerState _owner;
    };
  }

  class Mutex : public detail::OwnerTrackedMutexBase<Mutex>
  {
  public:
    Mutex()
    {
      ensureCreated();
    }

    explicit Mutex(DeferredCreateTag) noexcept
    {
    }

    ~Mutex()
    {
      if (_handle)
        vSemaphoreDelete(_handle);
    }

    Mutex(const Mutex &) = delete;
    Mutex &operator=(const Mutex &) = delete;
    Mutex(Mutex &&) = delete;
    Mutex &operator=(Mutex &&) = delete;

    bool valid() const noexcept { return _handle != nullptr; }

    bool ensureCreated()
    {
      if (!_handle)
        _handle = xSemaphoreCreateMutex();
      return _handle != nullptr;
    }

    SemaphoreHandle_t native_handle() const noexcept { return _handle; }

  private:
    friend class detail::OwnerTrackedMutexBase<Mutex>;

    bool takeImpl(TickType_t timeout)
    {
      return xSemaphoreTake(_handle, timeout) == pdTRUE;
    }

    void giveImpl()
    {
      xSemaphoreGive(_handle);
    }

    SemaphoreHandle_t _handle = nullptr;
  };

  class RecursiveMutex : public detail::OwnerTrackedMutexBase<RecursiveMutex>
  {
  public:
    RecursiveMutex()
    {
      ensureCreated();
    }

    explicit RecursiveMutex(DeferredCreateTag) noexcept
    {
    }

    ~RecursiveMutex()
    {
      if (_handle)
        vSemaphoreDelete(_handle);
    }

    RecursiveMutex(const RecursiveMutex &) = delete;
    RecursiveMutex &operator=(const RecursiveMutex &) = delete;
    RecursiveMutex(RecursiveMutex &&) = delete;
    RecursiveMutex &operator=(RecursiveMutex &&) = delete;

    bool valid() const noexcept { return _handle != nullptr; }

    bool ensureCreated()
    {
      if (!_handle)
        _handle = xSemaphoreCreateRecursiveMutex();
      return _handle != nullptr;
    }

    SemaphoreHandle_t native_handle() const noexcept { return _handle; }

  private:
    friend class detail::OwnerTrackedMutexBase<RecursiveMutex>;

    bool takeImpl(TickType_t timeout)
    {
      return xSemaphoreTakeRecursive(_handle, timeout) == pdTRUE;
    }

    void giveImpl()
    {
      xSemaphoreGiveRecursive(_handle);
    }

    SemaphoreHandle_t _handle = nullptr;
  };

  class BinarySemaphore
  {
  public:
    BinarySemaphore()
    {
      ensureCreated();
    }

    explicit BinarySemaphore(DeferredCreateTag) noexcept
    {
    }

    ~BinarySemaphore()
    {
      if (_handle)
        vSemaphoreDelete(_handle);
    }

    BinarySemaphore(const BinarySemaphore &) = delete;
    BinarySemaphore &operator=(const BinarySemaphore &) = delete;
    BinarySemaphore(BinarySemaphore &&) = delete;
    BinarySemaphore &operator=(BinarySemaphore &&) = delete;

    bool valid() const noexcept { return _handle != nullptr; }

    bool ensureCreated()
    {
      if (!_handle)
        _handle = xSemaphoreCreateBinary();
      return _handle != nullptr;
    }

    SemaphoreHandle_t native_handle() const noexcept { return _handle; }

    bool take(TickType_t timeout = portMAX_DELAY)
    {
      return ensureCreated() && xSemaphoreTake(_handle, timeout) == pdTRUE;
    }

    bool wait(TickType_t timeout = portMAX_DELAY)
    {
      return take(timeout);
    }

    void give()
    {
      configASSERT(_handle != nullptr);
      xSemaphoreGive(_handle);
    }

    void signal()
    {
      give();
    }

  private:
    SemaphoreHandle_t _handle = nullptr;
  };

  template <typename MutexType>
  using Lock = std::unique_lock<MutexType>;

  using FreertosLock = Lock<Mutex>;
  using RecursiveFreertosLock = Lock<RecursiveMutex>;

  template <typename MutexType>
  Lock<MutexType> tryLock(MutexType &mutex, TickType_t timeout = portMAX_DELAY)
  {
    if (!mutex.take(timeout))
      return {};
    return Lock<MutexType>(mutex, std::adopt_lock);
  }
}
