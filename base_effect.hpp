#pragma once

#include "object_handle.hpp"
#include "update_state.hpp"

namespace tano
{
  class GraphicsContext;


  class BaseEffect
  {
  public:

    BaseEffect(const string& instanceName, const string& config, u32 id);
    virtual ~BaseEffect() {}
    // Called before init, and then whenever config changes on disk
    virtual bool OnConfigChanged(const vector<char>& buf);
    virtual bool Init();
    virtual bool Show();
    virtual bool Hide();
    virtual bool Update(const UpdateState& state);
    virtual bool FixedUpdate(const FixedUpdateState& state);
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

    template <typename T>
    void SaveSettings(const T& settings)
    {
      OutputBuffer buf;
      Serialize(buf, settings);
      if (FILE* f = fopen(_configName.c_str(), "wt"))
      {
        fwrite(buf._buf.data(), 1, buf._ofs, f);
        fclose(f);
      }
    }

  protected:

    string _instanceName;
    u32 _id;

    TimeDuration _startTime, _endTime;
    bool _running;

    string _configName;
    GraphicsContext* _ctx;
    bool _firstTick;
  };
}
