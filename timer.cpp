#include "timer.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Timer::Timer()
{
}

//------------------------------------------------------------------------------
void Timer::Reset()
{
  _curTime = _startTime = TimeStamp::Now();
  if (_cycle.Ticks() > 0)
    _cycleTime = _curTime + _cycle;
}

//------------------------------------------------------------------------------
void Timer::Start()
{
  if (_running)
    return;

  _running = true;

  // check if we are resuming, or starting for the first time
  if (!_startTime.IsValid())
  {
    // first time
    _curTime = _startTime = TimeStamp::Now();
  }
  else
  {
    // resuming. curtime is set to when the timer was stopped, so we can
    // use this to adjust our start time like nothing happened :)
    TimeStamp now = TimeStamp::Now();
    TimeDuration delta = now - _curTime;
    _startTime += delta;
    _curTime = now;
  }

  // If the timer is set to loop, set the next end time
  if (_cycle.Ticks() > 0)
    _cycleTime = _curTime + _cycle;
}

//------------------------------------------------------------------------------
void Timer::Stop()
{
  if (!_running)
    return;

  _running = false;
  _curTime = TimeStamp::Now();
}

//------------------------------------------------------------------------------
void Timer::SetElapsed(TimeDuration elapsed)
{
  if (_running)
  {
    _startTime = TimeStamp::Now() - elapsed;
  }
  else
  {
    _startTime = _curTime - elapsed;
  }
}

//------------------------------------------------------------------------------
TimeDuration Timer::Peek() const
{
  u64 delta = ((_running ? TimeStamp::Now() : _curTime) - _startTime).Ticks();
  return TimeDuration(delta);
}

//------------------------------------------------------------------------------
TimeDuration Timer::Elapsed(TimeDuration* delta)
{
  TimeStamp prev = _curTime;
  if (_running)
    _curTime = TimeStamp::Now();

  if (delta)
    *delta = _curTime - prev;

  // check if the timer has looped
  if (_cycle.Ticks() > 0 && _curTime > _cycleTime)
    Reset();

  return _curTime - _startTime;
}

//------------------------------------------------------------------------------
bool Timer::IsRunning() const
{
  return _running;
}

//------------------------------------------------------------------------------
void Timer::SetCycle(const TimeDuration& cycle)
{
  _cycle = cycle;
  _cycleTime = _startTime + _cycle;
}
