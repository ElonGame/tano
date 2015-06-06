#include "free_list.hpp"

using namespace tano;

//------------------------------------------------------------------------------
FreeList::FreeList(void* start, void* end, size_t elementSize)
{
  // init each element, so it points to the next
  Node* startNode = (Node*)start;
  Node* endNode = (Node*)end;
  Node* cur = (Node*)startNode;
  size_t numElements = ((char*)end - (char*)start) / elementSize;

  for (size_t i = 0; i < numElements - 1; ++i)
  {
    cur->next = (Node*)((char*)cur + elementSize);
    cur = cur->next;
  }
  cur->next = nullptr;

  _head = startNode;
}

//------------------------------------------------------------------------------
void* FreeList::Alloc()
{
  Node* tmp = _head;
  _head = _head->next;
  return tmp;
}

//------------------------------------------------------------------------------
void FreeList::Free(void* ptr)
{
  Node* node = (Node*)ptr;
  node->next = _head;
  _head = node;
}
