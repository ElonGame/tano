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
}

//------------------------------------------------------------------------------
void Timer::Start()
{
  if (_flags.IsSet(Timer::TimerFlag::Running))
    return;

  _flags.Set(Timer::TimerFlag::Running);

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
}

//------------------------------------------------------------------------------
void Timer::Stop()
{
  if (!_flags.IsSet(Timer::TimerFlag::Running))
    return;

  _flags.Clear(Timer::TimerFlag::Running);
  _curTime = TimeStamp::Now();
}

//------------------------------------------------------------------------------
void Timer::SetElapsed(const TimeDuration& elapsed)
{
  if (_flags.IsSet(Timer::TimerFlag::Running))
    _startTime = TimeStamp::Now() - elapsed;
  else
    _startTime = _curTime - elapsed;
}

//------------------------------------------------------------------------------
void Timer::Adjust(const TimeDuration& delta)
{
  _startTime -= delta;
}

//------------------------------------------------------------------------------
TimeDuration Timer::Peek() const
{
  TimeStamp cur = _flags.IsSet(Timer::TimerFlag::Running) ? TimeStamp::Now() : _curTime;
  return cur - _startTime;
}

//------------------------------------------------------------------------------
TimeDuration Timer::Elapsed(TimeDuration* delta)
{
  TimeStamp prev = _curTime;
  if (_flags.IsSet(Timer::TimerFlag::Running))
    _curTime = TimeStamp::Now();

  if (delta)
    *delta = _curTime - prev;

  // check if the timer has looped
  TimeDuration elapsed = _curTime - _startTime;
  if (_flags.IsSet(Timer::TimerFlag::Looping) && elapsed > _cycle)
    Reset();

  return elapsed;
}

//------------------------------------------------------------------------------
bool Timer::IsRunning() const
{
  return _flags.IsSet(Timer::TimerFlag::Running);
}

//------------------------------------------------------------------------------
void Timer::SetCycle(const TimeDuration& cycle)
{
  _cycle = cycle;
}
