#pragma once
#include <vector>

namespace bristol
{
  bool LoadFile(const char* filename, std::vector<char>* buf);
  bool SaveFile(const char* filename, const void* buf, int len);
  bool FileExists(const char* filename);
  bool DirectoryExists(const char *name);
  time_t LastModification(const char* filename);
  std::string PathJoin(const char* root, const char* child);
}
