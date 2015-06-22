#pragma once

namespace tano
{
  struct StopWatch
  {
    StopWatch();
    void Start();
    double Stop();

    LARGE_INTEGER _frequency;
    LARGE_INTEGER _start;
  };

  struct AvgStopWatch
  {
    AvgStopWatch();
    void Start();
    double Stop();

    StopWatch _stopWatch;
    RollingAverage<double> _avg;
  };
}