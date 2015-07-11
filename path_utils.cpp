#include "path_utils.hpp"

using namespace tano;

//------------------------------------------------------------------------------
static string ReplaceAll(const string& str, char toReplace, char replaceWith)
{
  string res(str);
  for (size_t i = 0; i < res.size(); ++i)
  {
    if (res[i] == toReplace)
      res[i] = replaceWith;
  }
  return res;
}

//------------------------------------------------------------------------------
string Path::MakeCanonical(const string &str)
{
  // convert back slashes to forward
  return ReplaceAll(str, '\\', '/');
}

//------------------------------------------------------------------------------
Path::Path(const string& str)
{
  Init(str);
}

//------------------------------------------------------------------------------
void Path::Init(const string& str)
{
  _str = MakeCanonical(str);
  _extPos = _str.rfind(".");
  _finalSlashPos = str.rfind("/");
}


//------------------------------------------------------------------------------
Path Path::ReplaceExtension(const string& ext)
{
  return _extPos == string::npos ? Path(_str + "." + ext) : Path(string(_str, _extPos) + "." + ext);
}

//------------------------------------------------------------------------------
const string& Path::Str() const
{
  return _str;
}

//------------------------------------------------------------------------------
string Path::GetPath() const
{
  return _finalSlashPos == string::npos ? string() : _str.substr(0, _finalSlashPos + 1);
}

//------------------------------------------------------------------------------
string Path::GetExt() const
{
  return _extPos == string::npos ? string() : string(&_str[_extPos+1]);
}

//------------------------------------------------------------------------------
string Path::GetFilename() const
{
  return _finalSlashPos == string::npos ? _str : string(&_str[_finalSlashPos+1]);
}

//------------------------------------------------------------------------------
string Path::GetFilenameWithoutExt() const
{
  if (_str.empty())
    return _str;

  size_t filenamePos = _finalSlashPos == string::npos ? 0 : _finalSlashPos + 1;
  // 0123456
  // /tjong.ext
  // ^     ^--- _extPos
  // +--------- _finalSlashPos
  size_t end = _extPos == string::npos ? _str.size() : _extPos;
  return _str.substr(filenamePos, end - filenamePos);
}

//------------------------------------------------------------------------------
string Path::GetFullPathName(const char* p)
{
  char buf[MAX_PATH];
  ::GetFullPathNameA(p, MAX_PATH, buf, NULL);
  return buf;
}

//------------------------------------------------------------------------------
string Path::ReplaceExtension(const string& path, const string& ext)
{
  string res;
  if (!path.empty() && !ext.empty())
  {
    const char *dot = path.c_str();
    while (*dot && *dot++ != '.')
      ;

    if (*dot)
      res = string(path.c_str(), dot - path.c_str()) + ext;
    else
      res = path + (dot[-1] == '.' ? "" : ".") + ext;
  }

  return res;
}

//------------------------------------------------------------------------------
string Path::GetExt(const string &p)
{
  Path a(p);
  return a.GetExt();
}

//------------------------------------------------------------------------------
string Path::GetPath(const string& p)
{
  Path a(p);
  return a.GetPath();
}

//------------------------------------------------------------------------------
bool Path::IsAbsolute(const string& path)
{
  size_t len = path.size();
  return 
    len >= 1 && path[0] == '/' || 
    len >= 3 && path[1] == ':' && path[2] == '/';
}

//------------------------------------------------------------------------------
void Path::Split(const string& path, string* head, string* tail)
{
  // split path up to the last component
  Path p(path);
  if (p._finalSlashPos == string::npos)
  {
    // no final slash, so everything in tail
    *head = string();
    *tail = path;
    return;
  }

  *head = p._str.substr(0, p._finalSlashPos);
}

namespace tano
{
  //------------------------------------------------------------------------------
  string ReplaceExtension(const string& org, const string& newExt)
  {
    return StripExtension(org) + "." + newExt;
  }

  //------------------------------------------------------------------------------
  string StripExtension(const string& str)
  {
    size_t idx = str.rfind('.');
    if (idx == string::npos)
      return str;
    return string(str.c_str(), idx);
  }

  //------------------------------------------------------------------------------
  string GetExtension(const string& str)
  {
    size_t idx = str.rfind('.');
    if (idx == string::npos)
      return "";
    return string(str.c_str() + idx + 1, str.size() - idx);
  }

  //------------------------------------------------------------------------------
  void SplitPath(const char* path, string* drive, string* dir, string* fname, string* ext)
  {
    char drive_buf[_MAX_DRIVE];
    char dir_buf[_MAX_DIR];
    char fname_buf[_MAX_FNAME];
    char ext_buf[_MAX_EXT];
    _splitpath(path, drive_buf, dir_buf, fname_buf, ext_buf);
    if (drive) *drive = drive_buf;
    if (dir) *dir = dir_buf;
    if (fname) *fname = fname_buf;
    if (ext) *ext = ext_buf;
  }

  //------------------------------------------------------------------------------
  string Path::Join(const string& prefix, const string& suffix)
  {
    string res;
    res.reserve(prefix.size() + 1 + suffix.size());
    res.append(prefix);
    if (!prefix.empty() && prefix.back() != '\\' && prefix.back() != '/')
      res += '/';
    res.append(suffix);
    return MakeCanonical(res);
    return res;

  }

  //------------------------------------------------------------------------------
  string StripTrailingSlash(const string& str)
  {
    if (str.back() == '\\' || str.back() == '/')
      return string(str.data(), str.size() - 1);
    return str;
  }


}
