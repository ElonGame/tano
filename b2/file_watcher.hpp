#pragma once
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include "time_utils.hpp"

namespace bristol
{
  class FileWatcher
  {
  public:
    typedef std::function<bool(const std::string&)> cbFileChanged;
    typedef uint32_t WatchId;

    FileWatcher();
    void Tick();

    WatchId AddFileWatch(
        const std::string& filename,
        bool initialCallback,
        bool* initialResult,
        const cbFileChanged &cb);

    void RemoveFileWatch(WatchId id);

  private:

    struct CallbackContext
    {
      cbFileChanged cb;
      WatchId id;
    };

    struct WatchedFile
    {
      std::string filename;
      time_t lastModification;
      WatchId id;
      std::vector<CallbackContext> callbacks;
    };

    TimeStamp _lastTickTime;
    std::vector<std::string> _paths;
    std::vector<std::shared_ptr<WatchedFile>> _watchedFiles;
    std::unordered_map<WatchId, std::shared_ptr<WatchedFile>> _idToFile;
    uint32_t _nextId;

    uint32_t _watchFileOfs = 0;
  };
}
