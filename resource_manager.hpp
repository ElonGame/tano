#pragma once

#include "object_handle.hpp"

namespace tano
{
  typedef function<bool(const string&, void*)> cbFileChanged;
  struct AddFileWatchResult {
    FileWatcher::WatchId watchId;
    bool initialResult;
  };

#if WITH_UNPACKED_RESOUCES

  class ResourceManager
  {
  public:
    ResourceManager(const char* outputFilename);
    ~ResourceManager();

    static ResourceManager &Instance();
    static bool Create(const char* outputFilename, const char* appRoot);
    static bool Destroy();

    bool FileExists(const char* filename);
    __time64_t ModifiedDate(const char* filename);
    bool LoadFile(const char* filename, vector<char>* buf);
    bool LoadPartial(const char* filename, u32 ofs, u32 len, vector<char>* buf);
    bool LoadInplace(const char* filename, u32 ofs, u32 len, void* buf);

    // file is opened relateive to the app root
    FILE* OpenWriteFile(const char* filename);
    template <typename T>
    void WriteFile(FILE* f, const T& t)
    {
      WriteFile(f, (const char*)&t, sizeof(t));
    }
    void WriteFile(FILE* f, const char* buf, int len);
    void CloseFile(FILE* f);

    ObjectHandle LoadTexture(
        const char* filename,
        bool srgb = false,
        D3DX11_IMAGE_INFO* info = nullptr);
    ObjectHandle LoadTextureFromMemory(
        const char* buf,
        u32 len,
        bool srgb,
        D3DX11_IMAGE_INFO* info);

    void AddPath(const string &path);

    AddFileWatchResult AddFileWatch(
        const string& filename, 
        void* token,
        bool initial_callback,
        const cbFileChanged& cb);

    void RemoveFileWatch(FileWatcher::WatchId id);

    void Tick();

  private:
    string ResolveFilename(const char* filename, bool fullPath);

    FileWatcher _fileWatcher;

    vector<string> _paths;
    unordered_map<string, string> _resolvedPaths;

    string _outputFilename;

    struct FileInfo
    {
      FileInfo(const string &orgName, const string &resolvedName) 
        : orgName(orgName), resolvedName(resolvedName) {}

      bool operator<(const FileInfo &rhs) const
      {
        return make_pair(orgName, resolvedName) < make_pair(rhs.orgName, rhs.resolvedName);
      }
      string orgName;
      string resolvedName;
    };

    set<FileInfo> _readFiles;
    string _appRoot;

  };
#define RESOURCE_MANAGER ResourceManager::Instance()
#define RESOURCE_MANAGER_STATIC ResourceManager

#else

  class PackedResourceManager
  {
  public:
    PackedResourceManager(const char* resourceFile);

    static PackedResourceManager &Instance();
    static bool Create(const char* resourceFile);
    static bool Destroy();

    bool LoadFile(const char* filename, vector<char>* buf);
    bool LoadPartial(const char* filename, size_t ofs, size_t len, vector<char>* buf);
    bool LoadInplace(const char* filename, size_t ofs, size_t len, void* buf);
    ObjectHandle LoadTexture(
        const char* filename,
        const char* friendlyName = nullptr,
        bool srgb = false,
        D3DX11_IMAGE_INFO* info = nullptr);

    ObjectHandle LoadTextureFromMemory(const char* buf, size_t len, const char* friendly_name, bool srgb, D3DX11_IMAGE_INFO* info);

    FileWatcher::WatchId AddFileWatch(
      const string& filename,
      void* token,
      bool initial_callback,
      bool* initial_result,
      const cbFileChanged& cb);

  private:

    bool Init();
    bool LoadPackedFile(const char* filename, vector<char>* buf);
    bool LoadPackedInplace(const char* filename, size_t ofs, size_t len, void* buf);
    int HashLookup(const char* key);

    struct PackedFileInfo {
      int offset;
      int compressedSize;
      int finalSize;
    };

    vector<char> _fileBuffer;
    vector<int> _intermediateHash;
    vector<int> _finalHash;
    vector<PackedFileInfo> _fileInfo;

    static PackedResourceManager* _instance;
    string _resourceFile;
  };

#define RESOURCE_MANAGER PackedResourceManager::Instance()
#define RESOURCE_MANAGER_STATIC PackedResourceManager

#endif
}
