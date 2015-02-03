#pragma once

namespace tano
{
  class Timer
  {
  public:
    Timer();

    void Start();
    void Stop();
    void SetElapsed(TimeDuration t);
    TimeDuration Peek() const;
    TimeDuration Elapsed(TimeDuration* delta);
    bool IsRunning() const;
    void Reset();
    void SetCycle(const TimeDuration& t);

  private:
    TimeStamp _startTime;
    TimeStamp _curTime;
    TimeDuration _cycle;
    TimeStamp _cycleTime;
    bool _running = false;
    bool _useCycle = false;
  };
}