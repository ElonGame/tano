#include "output_buffer.hpp"
#include <algorithm>
#include <assert.h>

namespace parser
{
  OutputBuffer::OutputBuffer() : _ofs(0), _capacity(32), _buf(_capacity) {}

  void OutputBuffer::EnsureCapacity(size_t required)
  {
    if (_capacity - _ofs < required)
    {
      size_t newSize = 2 * std::max(required, _capacity);
      _buf.resize(newSize);
      _capacity = newSize;
    }
  }

  char* OutputBuffer::Cur() { return _buf.data() + _ofs; }

  void OutputBuffer::Advance(size_t distance)
  {
    _ofs += distance;
    if (_ofs > _capacity)
    {
      printf("** OUTPUTBUFFER DUMP**\n%s\n", _buf.data());
    }
    assert(_capacity >= _ofs);
  }
}
