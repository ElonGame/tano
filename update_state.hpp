#pragma once

namespace tano
{
  struct UpdateState
  {
    TimeDuration globalTime;
    TimeDuration localTime;
    TimeDuration delta;
    //s64 frequency;
    //s32 numTicks;
    //float ticksFraction;
    bool paused;
  };

  struct FixedUpdateState
  {
    TimeDuration globalTime;
    TimeDuration localTime;
    float delta;
    bool paused;
  };

}