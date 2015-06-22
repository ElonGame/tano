#include "arena_allocator.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
ArenaAllocator* ArenaAllocator::_instance;

//------------------------------------------------------------------------------
bool ArenaAllocator::Create(void* start, void* end)
{
  if (_instance)
    return true;

  _instance = new ArenaAllocator();
  return _instance->Init(start, end);
}

//------------------------------------------------------------------------------
void ArenaAllocator::Destroy()
{
  if (!_instance)
    return;

  delete exch_null(_instance);
}

//------------------------------------------------------------------------------
ArenaAllocator& ArenaAllocator::Instance()
{
  return *_instance;
}

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
void* ArenaAllocator::Alloc(u32 size)
{
  ScopedCriticalSection cs(&_cs);

  if (size + _idx > _capacity)
    return nullptr;

  void* res = _mem + _idx;
  _idx += size;
  return res;
}
