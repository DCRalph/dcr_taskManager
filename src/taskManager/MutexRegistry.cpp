#include "taskManager/MutexRegistry.h"

#include <mutex>

namespace RtosUtils
{
  namespace
  {
    std::mutex &registryMutex()
    {
      static std::mutex m;
      return m;
    }

    std::vector<MutexEntry> &registryStorage()
    {
      static std::vector<MutexEntry> entries;
      return entries;
    }
  }

  void registerMutex(FreeRtosRaii::Mutex &m, const char *name)
  {
    std::lock_guard<std::mutex> lock(registryMutex());
    auto &entries = registryStorage();
    for (const auto &e : entries)
    {
      if (e.mutex == &m)
        return;
    }
    entries.push_back({name, &m, false});
  }

  void registerMutex(FreeRtosRaii::RecursiveMutex &m, const char *name)
  {
    std::lock_guard<std::mutex> lock(registryMutex());
    auto &entries = registryStorage();
    for (const auto &e : entries)
    {
      if (e.mutex == &m)
        return;
    }
    entries.push_back({name, &m, true});
  }

  std::vector<MutexEntry> list()
  {
    std::lock_guard<std::mutex> lock(registryMutex());
    return registryStorage();
  }
}
