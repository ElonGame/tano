#pragma once

namespace tano
{
  //------------------------------------------------------------------------------
  class ArenaAllocator
  {
  public:

    static bool Create(void* start, void* end);
    static void Destroy();
    static ArenaAllocator& Instance();

    void NewFrame();
    void* Alloc(u32 size);

  private:
    ArenaAllocator();
    ~ArenaAllocator();

    bool Init(void* start, void* end);

    static ArenaAllocator* _instance;

    u8* _mem = nullptr;
    u32 _idx = 0;
    u32 _capacity = 0;

    CRITICAL_SECTION _cs;
  };

#define ARENA ArenaAllocator::Instance()
}