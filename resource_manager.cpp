#include "resource_manager.hpp"
#include "graphics.hpp"
#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

#if WITH_UNPACKED_RESOUCES

//------------------------------------------------------------------------------
static bool FileExists(const char* filename)
{
  if (_access(filename, 0) != 0)
    return false;

  struct _stat status;
  if (_stat(filename, &status) != 0)
    return false;

  return !!(status.st_mode & _S_IFREG);
}

//------------------------------------------------------------------------------
static string NormalizePath(const string &path, bool addTrailingSlash)
{
  string res(path);
  for (char& ch : res)
  {
    if (ch == '\\')
      ch = '/';
  }

  if (addTrailingSlash) {
    if (!res.empty() && res.back() != '/')
      res.push_back('/');
  }
  return res;
}
//------------------------------------------------------------------------------

static ResourceManager* g_instance;

//------------------------------------------------------------------------------
ResourceManager &ResourceManager::Instance()
{
  return *g_instance;
}

//------------------------------------------------------------------------------
bool ResourceManager::Create(const char* outputFilename, const char* appRoot)
{
  g_instance = new ResourceManager(outputFilename);
  g_instance->_appRoot = appRoot;
  return true;
}

//------------------------------------------------------------------------------
bool ResourceManager::Destroy()
{
  delete exch_null(g_instance);
  return true;
}

//------------------------------------------------------------------------------
ResourceManager::ResourceManager(const char* outputFilename)
  : _outputFilename(outputFilename)
{
  _paths.push_back("./");
}

//------------------------------------------------------------------------------
ResourceManager::~ResourceManager()
{
  if (!_outputFilename.empty())
  {
    FILE* f = fopen(_outputFilename.c_str(), "wt");
    for (auto it = begin(_readFiles); it != end(_readFiles); ++it)
    {
      fprintf(f, "%s\t%s\n", it->orgName.c_str(), it->resolvedName.c_str());
    }
    fclose(f);
  }
}

//------------------------------------------------------------------------------
void ResourceManager::AddPath(const string& path)
{
  _paths.push_back(NormalizePath(path, true));
}

//------------------------------------------------------------------------------
bool ResourceManager::LoadFile(const char* filename, vector<char>* buf)
{
  const string& fullPath = ResolveFilename(filename, true);
  if (fullPath.empty())
    return false;
  _readFiles.insert(FileInfo(filename, fullPath));

  return bristol::LoadFile(fullPath.c_str(), buf);
}

//------------------------------------------------------------------------------
bool ResourceManager::LoadPartial(
    const char* filename,
    u32 ofs,
    u32 len,
    vector<char>* buf)
{
  buf->resize(len);
  return LoadInplace(filename, ofs, len, buf->data());
}

//------------------------------------------------------------------------------
bool ResourceManager::LoadInplace(
    const char* filename,
    u32 ofs,
    u32 len,
    void* buf)
{
  const string& fullPath = ResolveFilename(filename, true);
  _readFiles.insert(FileInfo(filename, fullPath));

  ScopedHandle h(CreateFileA(fullPath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 
      FILE_ATTRIBUTE_NORMAL, NULL));
  if (h.handle() == INVALID_HANDLE_VALUE)
    return false;

  DWORD size = GetFileSize(h, NULL);
  if (ofs + len > size)
    return false;

  DWORD res;
  if (SetFilePointer(h, ofs, 0, FILE_BEGIN) == INVALID_SET_FILE_POINTER) 
  {
    return false;
  }

  if (!ReadFile(h, buf, len, &res, NULL)) 
  {
    DWORD err = GetLastError();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
bool ResourceManager::FileExists(const char* filename)
{
  return !ResolveFilename(filename, false).empty();
}

//------------------------------------------------------------------------------
__time64_t ResourceManager::ModifiedDate(const char* filename)
{
  struct _stat s;
  _stat(filename, &s);
  return s.st_mtime;
}

//------------------------------------------------------------------------------
string ResourceManager::ResolveFilename(const char* filename, bool fullPath)
{

  if (bristol::FileExists(filename))
  {
    if (fullPath)
    {
      char buf[MAX_PATH];
      GetFullPathNameA(filename, MAX_PATH, buf, NULL);
      return NormalizePath(buf, false);
    }
    else
    {
      return NormalizePath(filename, false);
    }
  }

  auto it = _resolvedPaths.find(filename);
  if (it != _resolvedPaths.end())
    return it->second;
  string res;
#if _DEBUG
  // warn about duplicates
  int count = 0;
  for (size_t i = 0; i < _paths.size(); ++i)
  {
    string cand(_paths[i] + filename);
    if (bristol::FileExists(cand.c_str()))
    {
      count++;
      if (res.empty())
        res = cand.c_str();
    }
  }
  if (count > 1)
  {
//    LOG_WARNING_LN("Multiple paths resolved for file: %s", filename);
  }
#else
  for (size_t i = 0; i < _paths.size(); ++i)
  {
    string cand(_paths[i] + filename);
    if (bristol::FileExists(cand.c_str()))
    {
      res = cand.c_str();
      break;
    }
  }
#endif
  if (!res.empty())
  {
    res = NormalizePath(res.c_str(), false);
    _resolvedPaths[filename] = res;
  }
  return res;
}

//------------------------------------------------------------------------------
AddFileWatchResult ResourceManager::AddFileWatch(
    const string& filename,
    bool initialCallback,
    const cbFileChanged &cb)
{
  AddFileWatchResult res;
  res.watchId = _fileWatcher.AddFileWatch(filename, nullptr, initialCallback, &res.initialResult, cb);
  return res;
}

//------------------------------------------------------------------------------
void ResourceManager::RemoveFileWatch(FileWatcher::WatchId id)
{
  _fileWatcher.RemoveFileWatch(id);
}

//------------------------------------------------------------------------------
void ResourceManager::Tick()
{
  _fileWatcher.Tick();
}

//------------------------------------------------------------------------------
ObjectHandle ResourceManager::LoadTexture(
    const char* filename,
    bool srgb,
    D3DX11_IMAGE_INFO* info)
{
  string fullPath = ResolveFilename(filename, true);
  _readFiles.insert(FileInfo(filename, fullPath));

  return GRAPHICS.LoadTexture(fullPath.c_str(), srgb, info);
}

//------------------------------------------------------------------------------
ObjectHandle ResourceManager::LoadTextureFromMemory(
    const char* buf,
    u32 len,
    bool srgb,
    D3DX11_IMAGE_INFO* info)
{
  return GRAPHICS.LoadTextureFromMemory(buf, len, srgb, info);
}

//------------------------------------------------------------------------------
FILE* ResourceManager::OpenWriteFile(const char* filename)
{
  FILE* f = fopen(PathJoin(_appRoot.c_str(), filename).c_str(), "wb");
  if (!f)
    return nullptr;

  return f;
}

//------------------------------------------------------------------------------
void ResourceManager::WriteFile(FILE* f, const char* buf, int len)
{
  fwrite(buf, len, 1, f);
}

//------------------------------------------------------------------------------
void ResourceManager::CloseFile(FILE* f)
{
  fclose(f);
}


#else

//------------------------------------------------------------------------------
#include "lz4/lz4.h"

using namespace std::tr1::placeholders;
using namespace std;

//------------------------------------------------------------------------------
static PackedResourceManager* g_instance;
static const int cMaxFileBufferSize = 16*  1024*  1024;

//------------------------------------------------------------------------------
static u32 FnvHash(u32 d, const char* str)
{
  if (d == 0)
    d = 0x01000193;

  while (true) {
    char c = *str++;
    if (!c)
      return d;
    d = ((d*  0x01000193) ^ c) & 0xffffffff;
  }
}

//------------------------------------------------------------------------------
struct PackedHeader
{
  int headerSize;
  int numFiles;
};

//------------------------------------------------------------------------------
PackedResourceManager &PackedResourceManager::Instance()
{
  return* g_instance;
}

//------------------------------------------------------------------------------
bool PackedResourceManager::Create(const char* outputFilename)
{
  g_instance = new PackedResourceManager(outputFilename);
  return g_instance->Init();
}

//------------------------------------------------------------------------------
bool PackedResourceManager::Destroy()
{
  delete exch_null(g_instance);
  return true;
}

//------------------------------------------------------------------------------
PackedResourceManager::PackedResourceManager(const char* resourceFile)
  : _resourceFile(resourceFile)
{
}

//------------------------------------------------------------------------------
bool PackedResourceManager::Init()
{
  BEGIN_INIT_SEQUENCE();
  FILE* f = fopen(_resourceFile.c_str(), "rb");
  INIT_FATAL(f);
  DEFER([&]{ fclose(f); });

  PackedHeader header;
  INIT(fread(&header, sizeof(header), 1, f) == 1);

  // read the perfect hash tables
  _intermediateHash.resize(header.numFiles);
  _finalHash.resize(header.numFiles);

  INIT(fread(&_intermediateHash[0], sizeof(int), header.numFiles, f) == header.numFiles);
  INIT(fread(&_finalHash[0], sizeof(int), header.numFiles, f) == header.numFiles);

  _fileInfo.resize(header.numFiles);
  INIT(fread(&_fileInfo[0], sizeof(PackedFileInfo), header.numFiles, f) == header.numFiles);

  int pos = ftell(f);
  fseek(f, 0, SEEK_END);
  int endPos = ftell(f);
  int dataSize = endPos - pos;
  fseek(f, pos, SEEK_SET);
  _fileBuffer.resize(dataSize);
  INIT(fread(&_fileBuffer[0], 1, dataSize, f) == dataSize);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
int PackedResourceManager::HashLookup(const char* key)
{
  int d = _intermediateHash[FnvHash(0, key) % _intermediateHash.size()];
  return d < 0 ? _finalHash[-d-1] : _finalHash[FnvHash(d, key) % _finalHash.size()];
}

//------------------------------------------------------------------------------
bool PackedResourceManager::LoadPackedFile(const char* filename, vector<char>* buf)
{
  PackedFileInfo* p = &_fileInfo[HashLookup(filename)];
  buf->resize(p->finalSize);
  int res = LZ4_uncompress(&_fileBuffer[p->offset], buf->data(), p->finalSize);
  return res == p->compressedSize;
}

//------------------------------------------------------------------------------
bool PackedResourceManager::LoadPackedInplace(const char* filename, size_t ofs, size_t len, void* buf)
{
  vector<char> tmp;
  if (!LoadPackedFile(filename, &tmp))
    return false;

  memcpy(buf, tmp.data() + ofs, len);
  return true;
}

//------------------------------------------------------------------------------
bool PackedResourceManager::LoadFile(const char* filename, vector<char>* buf)
{
  return LoadPackedFile(filename, buf);
}

//------------------------------------------------------------------------------
bool PackedResourceManager::LoadPartial(const char* filename, size_t ofs, size_t len, vector<char>* buf)
{
  // this is kinda cheesy..
  vector<char> tmp;
  if (!LoadPackedFile(filename, &tmp))
    return false;
  buf->resize(len);
  copy(begin(tmp) + ofs, begin(tmp) + ofs + len, begin(*buf));
  return true;
}

//------------------------------------------------------------------------------
bool PackedResourceManager::LoadInplace(const char* filename, size_t ofs, size_t len, void* buf)
{
  return LoadPackedInplace(filename, ofs, len, buf);
}

//------------------------------------------------------------------------------
ObjectHandle PackedResourceManager::LoadTexture(
    const char* filename,
    const char* friendlyName,
    bool srgb,
    D3DX11_IMAGE_INFO* info)
{
  vector<char> tmp;
  LoadPackedFile(filename, &tmp);
  return GRAPHICS.LoadTextureFromMemory(
    tmp.data(), 
    (u32)tmp.size(), 
    srgb, 
    info);
}

//------------------------------------------------------------------------------
ObjectHandle PackedResourceManager::LoadTextureFromMemory(
  const char* buf, size_t len, bool srgb, D3DX11_IMAGE_INFO* info)
{
  return GRAPHICS.LoadTextureFromMemory(buf, (u32)len, srgb, info);
}

//------------------------------------------------------------------------------
AddFileWatchResult PackedResourceManager::AddFileWatch(
    const string& filename,
    bool initialCallback,
    const cbFileChanged& cb)
{
  // Invoke the callback directly
  AddFileWatchResult res;
  res.watchId = 0;
  res.initialResult = cb(filename, 0);
  return res;
}

#endif
