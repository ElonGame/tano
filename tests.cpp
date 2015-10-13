#if WITH_TESTS

#include "circular_buffer.hpp"
#include "fixed_deque.hpp"

using namespace tano;

#include <assert.h>

//------------------------------------------------------------------------------
bool EvalTest()
{
  eval::Environment env;
  env.functions["test"] = eval::UserFunction{2, [](eval::Evaluator* eval) {
    float b = eval->PopValue();
    float a = eval->PopValue();
    eval->PushValue(a + b);
  }};

  env.constants["a"] = 10;

  eval::Evaluator ss;
  float res = ss.EvaluateFromString("test(1 / 10, 2 / 10) * a", &env);
  assert(res == 3.0f);

  return true;
}

//------------------------------------------------------------------------------
bool BufferTest()
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

//------------------------------------------------------------------------------
bool DequeTest()
{
  static const int N = 4;
  int buf[N];
  FixedDequeue<int> d(buf, N);

  assert(d.IsEmpty());

  d.PushFront(10);
  d.PushBack(20);
  assert(d.Front() == 10);
  d.PopFront();
  assert(d.Front() == 20);
  d.PopFront();
  assert(d.IsEmpty());

  for (int i = 0; i < N; ++i)
  {
    if (i % 2)
      d.PushBack(i);
    else
      d.PushFront(i);
  }

  assert(d.IsFull());

  for (int i = 0; i < N; ++i)
  {
    if (i % 2)
    {
      int v = d.Back();
      assert(v == N - i);
      d.PopBack();
    }
    else
    {
      int v = d.Front();
      assert(v == N - 2 - i);
      d.PopFront();
    }
  }

  assert(d.IsEmpty());

  return true;
}

//------------------------------------------------------------------------------
static bool bufferTestPassed = BufferTest();
static bool dequeTest1Passed = DequeTest();
static bool evalTestPassed = EvalTest();

#endif