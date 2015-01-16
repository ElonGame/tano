#pragma once

namespace test {

struct OutputBuffer
{
  OutputBuffer();
  void EnsureCapacity(size_t required);
  char* Cur();
  void Advance(size_t distance);

  size_t _ofs;
  size_t _capacity;
  vector<char> _buf;
};

}
