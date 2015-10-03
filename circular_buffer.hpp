#pragma once

namespace tano
{
  template<typename T>
  struct CircularBuffer
  {
    CircularBuffer(void* start, void *end)
    {
      buf = (T*)start;
      capacity = (u32)((T*)end - (T*)start);
    }

    void Push(const T& element)
    {
      ++used;
      buf[WrappedInc(writePtr)] = element;
    }

    T Pop()
    {
      --used;
      return buf[WrappedInc(readPtr)];
    }

    bool IsFull()
    {
      return used == capacity;
    }

    bool IsEmpty()
    {
      return used == 0;
    }

    u32 WrappedInc(u32& value)
    {
      u32 old = value;
      value = (value + 1) % capacity;
      return old;
    }

    T* buf;
    u32 capacity;
    u32 used = 0;
    u32 readPtr = 0;
    u32 writePtr = 0;
  };
}
