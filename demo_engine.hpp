#pragma once

#include "graphics.hpp"
#include "timer.hpp"
#include "property_manager.hpp"
#include "generated/demo.types.hpp"
#include "base_effect.hpp"

namespace tano
{

  class BaseEffect;
  typedef function<BaseEffect*(const char*, const char*, u32)> EffectFactory;

  class DemoEngine
  {
  public:
    static void Create();
    static void Destroy();

    bool Init(const char* config, HINSTANCE instance);
    void RegisterFactory(const string& type, const EffectFactory& factory);

    bool Start();
    void SetPaused(bool pause);
    bool Paused() const;

    void AdjustPos(const TimeDuration& delta);
    void SetPos(const TimeDuration& pos);
    TimeDuration Pos();

    TimeDuration Duration() const;
    void SetDuration(const TimeDuration& duration);
    bool Tick();
    bool IsDone();

    void WndProc(UINT message, WPARAM wParam, LPARAM lParam);
    double GetRow() const;

    //static bool IsInitialized() { return  _instance != nullptr; }

#if WITH_IMGUI
    PropertyManager& GetPropertyManager() { return _propertyManager; }
#endif

  private:

    DemoEngine();
    ~DemoEngine();

    void ReclassifyEffects();
    void KillEffects();

    void UpdateEffects();
    BaseEffect* FindEffectByName(const string &name);
    bool ApplySettings(const DemoSettings& settings);

    static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    deque<BaseEffect*> _activeEffects;
    deque<BaseEffect*> _inactiveEffects;
    deque<BaseEffect*> _expiredEffects;
    vector<BaseEffect*> _effects;
    int _cur_effect;
    TimeDuration _duration;
    TimeDuration _accumulated;

    Timer _demoTimer;
    Timer _globalTimer;

    string _configFile;
    DemoSettings _settings;
    unordered_map<string, EffectFactory> _effectFactories;
    u32 _nextEffectId;

#if WITH_IMGUI
    PropertyManager _propertyManager;
#endif

#if WITH_MUSIC
    HSTREAM _stream = 0;
#endif

    UpdateState _initialState;
    FixedUpdateState _initialFixedState;
    BaseEffect* _forceEffect = nullptr;

    double _updatedAcc = 0;
    bool _initForceEffect = false;
  };

  extern DemoEngine* g_DemoEngine;
#define PROPERTIES g_DemoEngine->GetPropertyManager()

}
