#pragma once

#include "object_handle.hpp"
#include "update_state.hpp"
#include "camera.hpp"

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

    virtual const char* GetName() = 0;
    const string& InstanceName() const { return _instanceName; }
    TimeDuration StartTime() const { return _startTime; }
    TimeDuration EndTime() const { return _endTime; }
    bool Running() const;
    void SetRunning(bool b);
    void SetDuration(TimeDuration startTime, TimeDuration endTime);
    void SetStartTime(TimeDuration startTime);

    u32 GetId() const { return _id; }

    void CommonUpdate(float deltaTime);

#if WITH_IMGUI
    void RegisterParameters();
    virtual void RenderParameterSet() {}
    virtual void SaveParameterSet(bool inc) {}
#endif
    
    void LoadParameterVersion(int version);

    template <typename T>
    void SaveSettings(const T& settings, bool increment = false)
    {
      string filename = _configName;

      if (increment)
      {
        int version = 0;
        if (!_configFileVersions.empty())
        {
          for (int v : _configFileVersions)
            version = max(version, v);
        }

        version += 1;
        char buf[MAX_PATH];
        sprintf(buf, "%s.%d", _configName.c_str(), version);
        filename = buf;
      }
      else if (_currentConfigVersion != -1)
      {
        filename = _configVersionToFilename[_currentConfigVersion];
      }

      OutputBuffer buf;
      Serialize(buf, settings);
      if (FILE* f = fopen(filename.c_str(), "wt"))
      {
        fwrite(buf._buf.data(), 1, buf._ofs, f);
        fclose(f);
      }

      RegisterParameters();
    }

  protected:

    string _instanceName;
    u32 _id;

    TimeDuration _startTime, _endTime;
    bool _running;

    string _configName;
    GraphicsContext* _ctx;
    bool _firstTick;

    int _currentConfigVersion = -1;
    vector<int> _configFileVersions;
    unordered_map<int, string> _configVersionToFilename;

    FreeflyCamera _freeflyCamera;
  };
}
