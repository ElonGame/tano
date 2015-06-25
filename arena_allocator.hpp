#pragma once

namespace tano
{
  //------------------------------------------------------------------------------
  class ArenaAllocator
  {
  public:
    ArenaAllocator();
    ~ArenaAllocator();

    bool Init(void* start, void* end);
    void NewFrame();
    void* Alloc(u32 size, u32 alignment = 16);

  private:

    u8* _mem = nullptr;
    u32 _idx = 0;
    u32 _capacity = 0;

    CRITICAL_SECTION _cs;
  };

  extern ArenaAllocator g_ScratchMemory;
  extern ArenaAllocator g_GlobalMemory;
}
