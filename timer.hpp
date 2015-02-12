#pragma once

namespace tano
{
  class Timer
  {
  public:
    Timer();

    void Start();
    void Stop();
    void Adjust(const TimeDuration& delta);
    void SetElapsed(const TimeDuration& elapsed);
    TimeDuration Peek();
    TimeDuration Elapsed(TimeDuration* delta);
    bool IsRunning() const;
    void Reset();

  private:

    void Update();

    // Note, looping isn't implemented yet
    struct TimerFlag
    {
      enum Enum { Running = 1 << 0, Looping = 1 << 1, };
      struct Bits { u32 running : 1; u32 looping : 1; };
    };

    typedef Flags<TimerFlag> TimerFlags;

    TimeDuration _timeDiff;
    TimeDuration _userDiff;

    TimeDuration _curTime;
    TimeDuration _prevElapsed;
    TimeStamp _lastTick;
    TimerFlags _flags;
  };
}
