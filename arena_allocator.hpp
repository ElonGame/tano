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
    template<typename T> T* Alloc(u32 count, u32 alignment = 16)
    {
      return (T*)Alloc(count * sizeof(T), alignment);
    }

    template<typename T> T* New(u32 count, u32 alignment = 16)
    {
      T* mem = (T*)Alloc(count * sizeof(T), alignment);
      for (u32 i = 0; i < count; ++i)
        new(mem + i)T();
      return mem;
    }

  private:

    u8* _mem = nullptr;
    u32 _idx = 0;
    u32 _capacity = 0;

    CRITICAL_SECTION _cs;
  };

  extern ArenaAllocator g_ScratchMemory;
  extern ArenaAllocator g_GlobalMemory;
}
