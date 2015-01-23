#pragma once

#include "object_handle.hpp"

namespace tano
{
  class DeferredContext;

  struct UpdateState
  {
    TimeDuration globalTime;
    TimeDuration localTime;
    TimeDuration delta;
    s64 frequency;
    s32 numTicks;
    float ticksFraction;
    bool paused;
  };

  class Effect
  {
  public:

    Effect(const string& name, u32 id);
    virtual ~Effect();
    virtual bool Init(const char* configFile);
    virtual bool Show();
    virtual bool Hide();
    virtual bool Update(const UpdateState& state);
    virtual bool Render();
    virtual bool Close();

    const string& Name() const { return _name; }
    TimeDuration StartTime() const { return _startTime; }
    TimeDuration EndTime() const { return _endTime; }
    bool Running() const;
    void SetRunning(bool b);
    void SetDuration(TimeDuration startTime, TimeDuration endTime);
    void SetStartTime(TimeDuration startTime);

    u32 GetId() const { return _id; }

  protected:

    string _name;
    u32 _id;

    TimeDuration _startTime, _endTime;
    bool _running;

    DeferredContext* _ctx;
    bool _firstTick;
  };
}
