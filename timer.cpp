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
  _curTime = TimeDuration::Seconds(0);
  _lastTick = TimeStamp::Now();
}

//------------------------------------------------------------------------------
void Timer::Start()
{
  // If already running, just return
  if (_flags.CheckAndSet(Timer::TimerFlag::Running))
    return;

  _lastTick = TimeStamp::Now();
}

//------------------------------------------------------------------------------
void Timer::Stop()
{
  // If not running, just return
  if (!_flags.CheckAndClear(Timer::TimerFlag::Running))
    return;
}

//------------------------------------------------------------------------------
void Timer::Update()
{
  TimeStamp now = TimeStamp::Now();
  if (_flags.IsSet(Timer::TimerFlag::Running))
  {
    TimeDuration delta = now - _lastTick;
    _curTime += delta;
  }
  _lastTick = now;

  _curTime += _userDiff;
  _curTime.Clamp();
  _userDiff = TimeDuration::Seconds(0);
}

//------------------------------------------------------------------------------
void Timer::SetElapsed(const TimeDuration& elapsed)
{
  // Store the diff between the current time and the given time as a user_diff
  _userDiff = elapsed - _curTime;
}

//------------------------------------------------------------------------------
void Timer::Adjust(const TimeDuration& delta)
{
  _userDiff = delta;
}

//------------------------------------------------------------------------------
TimeDuration Timer::Peek()
{
  Update();
  return _curTime;
}

//------------------------------------------------------------------------------
TimeDuration Timer::Elapsed(TimeDuration* delta)
{
  Update();
  if (delta)
    *delta = _curTime - _prevElapsed;

  _prevElapsed = _curTime;
  return _curTime;
}

//------------------------------------------------------------------------------
bool Timer::IsRunning() const
{
  return _flags.IsSet(Timer::TimerFlag::Running);
}