#pragma once

#include <taskManager/FreeRtosRaii.h>
#include <vector>

namespace RtosUtils
{
  struct MutexEntry
  {
    const char *name;
    void *mutex;
    bool recursive;
  };

  void registerMutex(FreeRtosRaii::Mutex &m, const char *name);
  void registerMutex(FreeRtosRaii::RecursiveMutex &m, const char *name);

  std::vector<MutexEntry> list();
}
