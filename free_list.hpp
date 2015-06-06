#pragma once

namespace tano
{
  struct FreeList
  {
    FreeList(void* start, void* end, size_t elementSize);

    void* Alloc();
    void Free(void*);

    struct Node
    {
      Node* next;
    };

    Node* _head;
  };

}