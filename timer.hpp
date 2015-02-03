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
    TimeDuration Peek() const;
    TimeDuration Elapsed(TimeDuration* delta);
    bool IsRunning() const;
    void Reset();
    void SetCycle(const TimeDuration& t);

  private:

    struct TimerFlag
    {
      enum Enum { Running = 1 << 0, Looping = 1 << 1, };
      struct Bits { u32 running : 1; u32 looping : 1; };
    };

    typedef Flags<TimerFlag> TimerFlags;

    TimeStamp _startTime;
    TimeStamp _curTime;
    TimeDuration _cycle;
    TimerFlags _flags;
  };
}