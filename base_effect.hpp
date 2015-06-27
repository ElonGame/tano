#pragma once

#include "object_handle.hpp"
#include "update_state.hpp"

namespace tano
{
  class GraphicsContext;


  class BaseEffect
  {
  public:

    BaseEffect(const string& instanceName, u32 id);
    virtual ~BaseEffect() {}
    virtual bool Init(const char* configFile);
    virtual bool Show();
    virtual bool Hide();
    virtual bool Update(const UpdateState& state);
    virtual bool Render();
    virtual bool Close();
    virtual bool InitAnimatedParameters();
    const string& InstanceName() const { return _instanceName; }
    TimeDuration StartTime() const { return _startTime; }
    TimeDuration EndTime() const { return _endTime; }
    bool Running() const;
    void SetRunning(bool b);
    void SetDuration(TimeDuration startTime, TimeDuration endTime);
    void SetStartTime(TimeDuration startTime);

    u32 GetId() const { return _id; }

  protected:

    string _instanceName;
    u32 _id;

    TimeDuration _startTime, _endTime;
    bool _running;

    GraphicsContext* _ctx;
    bool _firstTick;
    string _configName;
  };
}
