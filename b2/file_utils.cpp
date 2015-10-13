#include "file_utils.hpp"
#include "utils.hpp"

#include <sys/stat.h>
#ifdef _WIN32
#include <io.h>
#endif

using namespace std;

namespace bristol
{
  //------------------------------------------------------------------------------
  string PathJoin(const char* root, const char* child)
  {
    string res(root);
    if (res.size() > 0)
    {
      char ch = res.back();
      if (ch != '\\' && ch != '/')
        res.append("\\");
    }

    res.append(child);
    return res;
  }

  //------------------------------------------------------------------------------
  bool LoadFile(const char* filename, vector<char> *buf)
  {
#ifdef _WIN32
    ScopedHandle h(CreateFileA(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL));
    if (!h)
      return false;

    DWORD size = GetFileSize(h, NULL);
    if (size > 0)
    {
      buf->resize(size);
      DWORD res;
      if (!ReadFile(h, &(*buf)[0], size, &res, NULL)) 
        return false;
    }
    return true;
#else
    return false;
#endif
  }

  //------------------------------------------------------------------------------
  bool FileExists(const char* filename)
  {
#ifdef _WIN32
    struct _stat status;
    return _access(filename, 0) == 0 && _stat(filename, &status) == 0 && (status.st_mode & _S_IFREG);
#else
    return false;
#endif
  }

  //------------------------------------------------------------------------------
  bool DirectoryExists(const char* name)
  {
#ifdef _WIN32
    struct _stat status;
    return _access(name, 0) == 0 && _stat(name, &status) == 0 && (status.st_mode & _S_IFDIR);
#else
    return false;
#endif
  }

  //------------------------------------------------------------------------------
  bool SaveFile(const char *filename, const void *buf, int len)
  {
#pragma warning(suppress: 4996)
    FILE *f = fopen(filename, "wb");
    if (!f)
      return false;

    DEFER([=]() {
      fclose(f);
    });

    return fwrite(buf, len, 1, f) == len;
  }

  //------------------------------------------------------------------------------
  time_t LastModification(const char* filename)
  {
    struct stat s;
    stat(filename, &s);
    return s.st_mtime;
  }

  //------------------------------------------------------------------------------
}
