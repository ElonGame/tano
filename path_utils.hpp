#pragma once

namespace tano
{
  class Path
  {
  public:
    Path(const string& str);
    void Init(const string &str);
    Path ReplaceExtension(const string& ext);
    const string& Str() const;
    string GetPath() const;
    string GetExt() const;
    string GetFilename() const;
    string GetFilenameWithoutExt() const;

    static string MakeCanonical(const string& str);
    static string GetFullPathName(const char* p);
    static string ReplaceExtension(const string& path, const string& ext);
    static string GetPath(const string& p);
    static string GetExt(const string& p);

    static bool IsAbsolute(const string& path);

    static string Join(const string& prefix, const string& suffix);

    static void Split(const string& path, string* head, string* tail);

  private:
    string _str;
    size_t _finalSlashPos = string::npos;     // points to the last '/'
    size_t _extPos = string::npos;            // points to the '.'
  };

  string ReplaceExtension(const string& org, const string& new_ext);
  string StripExtension(const string& str);
  string StripTrailingSlash(const string& str);
  string GetExtension(const string& str);
  void SplitPath(const char* path, string* drive, string* dir, string* fname, string* ext);
}

