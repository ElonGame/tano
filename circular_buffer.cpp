#include "circular_buffer.hpp"

using namespace tano;

#if WITH_TESTS
#include <assert.h>

bool BufferTest1()
{
  const int N = 4;
  int mem[N];
  CircularBuffer<int> buf1(mem, mem + N);

  for (int j = 0; j < 3; ++j)
  {
    assert(buf1.IsEmpty());

    for (int i = 0; i < N; ++i)
      buf1.Push(i);

    assert(buf1.IsFull());

    for (int i = 0; i < N; ++i)
    {
      int a = buf1.Pop();
      assert(a == i);
    }
  }

  return true;
}

static bool bufferTest1Passed = BufferTest1();

#endif