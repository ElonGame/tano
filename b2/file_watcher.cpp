#include <algorithm>
#include "file_watcher.hpp"
#include "file_utils.hpp"

using namespace std;
using namespace bristol;

//------------------------------------------------------------------------------
FileWatcher::FileWatcher()
    : _nextId(1)
{
}

//------------------------------------------------------------------------------
FileWatcher::WatchId FileWatcher::AddFileWatch(
    const string& filename,
    bool initialCallback,
    bool* initialResult,
    const cbFileChanged &cb)
{
  shared_ptr<WatchedFile> w;
  for (shared_ptr<WatchedFile>& f : _watchedFiles)
  {
    if (f->filename == filename)
    {
      w = f;
      break;
    }
  }

  if (!w)
  {
    // the watch isn't already present, so add it
    w = make_shared<WatchedFile>(WatchedFile{ filename, LastModification(filename.c_str()) });
    _watchedFiles.push_back(w);
  }

  WatchId id = _nextId++;
  w->callbacks.push_back({cb, id});
  _idToFile[id] = w;

  // Check if we should invoke the callback directly
  if (initialCallback)
  {
    bool res = cb(filename);
    if (initialResult)
      *initialResult = res;
  }

  return id;
}

//------------------------------------------------------------------------------
void FileWatcher::RemoveFileWatch(WatchId id)
{
  auto it = _idToFile.find(id);
  if (it == _idToFile.end())
    return;

  shared_ptr<WatchedFile>& w = it->second;
  auto& callbacks = w->callbacks;
  callbacks.erase(
    std::remove_if(callbacks.begin(), callbacks.end(), [=](const CallbackContext& ctx) { return ctx.id == id; }), 
    callbacks.end());
}

//------------------------------------------------------------------------------
void FileWatcher::Tick()
{
  int MAX_FILES_PER_TICK = 100;

  TimeStamp now = TimeStamp::Now();
  if (!_lastTickTime.IsValid() || (now - _lastTickTime) > TimeDuration::Seconds(1))
  {
    int num = min((int)_watchedFiles.size(), MAX_FILES_PER_TICK);
    for (int i = 0; i < num; ++i)
    {
      shared_ptr<WatchedFile>& f = _watchedFiles[(i+_watchFileOfs) % num];
      const string& filename = f->filename;
      time_t lastModification = LastModification(filename.c_str());
      if (lastModification > f->lastModification)
      {
        // file has been modified, so call all the callbacks
        for (const CallbackContext& ctx : f->callbacks)
        {
          ctx.cb(filename);
        }

        f->lastModification = lastModification;
      }
    }
    _watchFileOfs += num;
  }
}
