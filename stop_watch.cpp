#include "stop_watch.hpp"

using namespace tano;

StopWatch::StopWatch()
{
  QueryPerformanceFrequency(&_frequency);
}

void StopWatch::Start()
{
  QueryPerformanceCounter(&_start);
}

double StopWatch::Stop()
{
  LARGE_INTEGER tmp;
  QueryPerformanceCounter(&tmp);

  return (double)(tmp.QuadPart - _start.QuadPart) / _frequency.QuadPart;
}
