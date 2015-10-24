#include "filewatcher_win32.hpp"
#include "path_utils.hpp"

using namespace tano;
using namespace bristol;

// FILE_NOTIFY_CHANGE_FILE_NAME is needed because photoshop doesn't modify the file directly,
// instead it saves a temp file, and then deletes/renames
static DWORD FILE_NOTIFY_FLAGS = FILE_NOTIFY_CHANGE_FILE_NAME | FILE_NOTIFY_CHANGE_LAST_WRITE;

//------------------------------------------------------------------------------
FileWatcherWin32::~FileWatcherWin32()
{
  for (auto kv : _watchesByDir)
  {
    WatchedDir* dir = kv.second;
    CloseHandle(dir->dirHandle);
    CloseHandle(dir->overlapped.hEvent);
    SeqDelete(&dir->callbacks);
    delete dir;
  }
  _watchesByDir.clear();
}

//------------------------------------------------------------------------------
FileWatcherWin32::AddFileWatchResult FileWatcherWin32::AddFileWatch(
  const string& filename, bool initialCallback, const cbFileChanged& cb)
{
  // split filename into dir/file
  string head, tail;
  string canonical = Path::MakeCanonical(filename);
  Path::Split(canonical, &head, &tail);

  // check if the directory is already open
  WatchedDir* dir = nullptr;
  auto it = _watchesByDir.find(head);
  if (it == _watchesByDir.end())
  {
    dir = new WatchedDir();
    dir->dirHandle = ::CreateFileA(
      head.c_str(),
      FILE_LIST_DIRECTORY,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      NULL,
      OPEN_EXISTING,
      FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OVERLAPPED,
      NULL);

    ZeroMemory(&dir->overlapped, sizeof(OVERLAPPED));
    dir->overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    BOOL res = ReadDirectoryChangesW(
      dir->dirHandle, _buf, BUF_SIZE, FALSE, FILE_NOTIFY_FLAGS, NULL, &dir->overlapped, NULL);
    if (!res)
    {
      delete dir;
      return AddFileWatchResult();
    }

    _watchesByDir[head] = dir;
  }
  else
  {
    dir = it->second;
  }

  dir->callbacks.push_back(new CallbackContext{ filename, tail, cb, _nextId });

  // Do the initial calback if requested
  AddFileWatchResult res;
  if (initialCallback)
  {
    res.initialResult = cb(filename);
    res.watchId = _nextId;
  }

  _nextId++;
  return res;
}

//------------------------------------------------------------------------------
void FileWatcherWin32::RemoveFileWatch(WatchId id)
{
  for (auto itDir = _watchesByDir.begin(); itDir != _watchesByDir.end(); )
  {
    WatchedDir* dir = itDir->second;
    for (auto itFile = dir->callbacks.begin(); itFile != dir->callbacks.end(); )
    {
      if ((*itFile)->id == id)
      {
        delete *itFile;
        itFile = dir->callbacks.erase(itFile);
      }
      else
      {
        ++itFile;
      }
    }

    if (dir->callbacks.empty())
    {
      CloseHandle(dir->dirHandle);
      CloseHandle(dir->overlapped.hEvent);
      delete dir;
      itDir = _watchesByDir.erase(itDir);
    }
    else
    {
      ++itDir;
    }
  }
}

//------------------------------------------------------------------------------
void FileWatcherWin32::Tick()
{
  // Check if any of the recently changed files have been idle for long enough to
  // call their callbacks
  u32 now = timeGetTime();

  for (auto it = _lastUpdate.begin(); it != _lastUpdate.end();)
  {
    const CallbackContext* ctx = it->first;
    u32 lastUpdate = it->second;
    if (now - lastUpdate > 1000)
    {
      ctx->cb(ctx->fullPath);
      it = _lastUpdate.erase(it);
    }
    else
    {
      it++;
    }
  }

  for (auto kv : _watchesByDir)
  {
    WatchedDir* dir = kv.second;
    DWORD bytesTransferred = 0;
    if (GetOverlappedResult(dir->dirHandle, &dir->overlapped, &bytesTransferred, FALSE))
    {
      char* ptr = _buf;
      while (true)
      {
        FILE_NOTIFY_INFORMATION* fni = (FILE_NOTIFY_INFORMATION*)ptr;

        string filename;
        wide_char_to_utf8(fni->FileName, fni->FileNameLength / 2, &filename);
        filename = Path::MakeCanonical(filename);

        // Check if this file is being watched, and update its last modified
        for (const CallbackContext* ctx : dir->callbacks)
        {
          if (ctx->filename == filename)
            _lastUpdate[ctx] = now;
        }

        if (fni->NextEntryOffset == 0)
          break;
        ptr += fni->NextEntryOffset;
      }

      // Reset the event, and reapply the watch
      ResetEvent(dir->overlapped.hEvent);
      ReadDirectoryChangesW(
        dir->dirHandle, _buf, BUF_SIZE, FALSE, FILE_NOTIFY_FLAGS, NULL, &dir->overlapped, NULL);
    }
  }
}
