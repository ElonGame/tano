#include "stop_watch.hpp"

using namespace tano;

//------------------------------------------------------------------------------
StopWatch::StopWatch()
{
  QueryPerformanceFrequency(&_frequency);
}

//------------------------------------------------------------------------------
void StopWatch::Start()
{
  QueryPerformanceCounter(&_start);
}

//------------------------------------------------------------------------------
double StopWatch::Stop()
{
  LARGE_INTEGER tmp;
  QueryPerformanceCounter(&tmp);

  return (double)(tmp.QuadPart - _start.QuadPart) / _frequency.QuadPart;
}

//------------------------------------------------------------------------------
AvgStopWatch::AvgStopWatch()
  : _avg(100)
{
}

//------------------------------------------------------------------------------
void AvgStopWatch::Start()
{
  _stopWatch.Start();
}

//------------------------------------------------------------------------------
double AvgStopWatch::Stop()
{
  _avg.AddSample(_stopWatch.Stop());
  return _avg.GetAverage();
}
