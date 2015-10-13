#pragma once

namespace parser
{

  struct OutputBuffer
  {
    OutputBuffer();
    void EnsureCapacity(size_t required);
    char* Cur();
    void Advance(size_t distance);

    size_t _ofs;
    size_t _capacity;
    std::vector<char> _buf;
  };
}
