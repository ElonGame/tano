#pragma once

namespace tano
{
  struct FileWatcherWin32
  {
    ~FileWatcherWin32();

    typedef int WatchId;
    typedef function<bool(const string&)> cbFileChanged;

    struct AddFileWatchResult
    {
      WatchId watchId = -1;
      bool initialResult = true;
    };

    AddFileWatchResult AddFileWatch(const string& filename, bool initialCallback, const cbFileChanged& cb);
    void RemoveFileWatch(WatchId id);

    void Tick();

  private:

    struct CallbackContext
    {
      string fullPath;
      string filename;
      cbFileChanged cb;
      WatchId id;
    };

    struct WatchedDir
    {
      HANDLE dirHandle;
      OVERLAPPED overlapped;
      vector<CallbackContext*> callbacks;
    };

    unordered_map<string, WatchedDir*> _watchesByDir;

    unordered_map<const CallbackContext*, u32> _lastUpdate;

    uint32_t _nextId = 0;

    enum { BUF_SIZE = 16 * 1024 };
    char _buf[BUF_SIZE];
  };
}
