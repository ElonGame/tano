#include "arena_allocator.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
ArenaAllocator::ArenaAllocator()
{
  InitializeCriticalSection(&_cs);
}

//------------------------------------------------------------------------------
ArenaAllocator::~ArenaAllocator()
{
  DeleteCriticalSection(&_cs);
}

//------------------------------------------------------------------------------
bool ArenaAllocator::Init(void* start, void* end)
{
  _mem = (u8*)start;
  _capacity = (u32)((u8*)end - _mem);
  return true;
}

//------------------------------------------------------------------------------
void ArenaAllocator::NewFrame()
{
  _idx = 0;
}

//------------------------------------------------------------------------------
void* ArenaAllocator::Alloc(u32 size, u32 alignment)
{
  ScopedCriticalSection cs(&_cs);

  // Calc padding needed for requested alignment
  u32 mask = alignment - 1;
  u32 padding = (alignment - (_idx & mask)) & mask;

  u32 alignedSize = size + padding;
  if (alignedSize + _idx > _capacity)
    return nullptr;

  u8* res = _mem + _idx + padding;
  _idx += alignedSize;
  return res;
}
