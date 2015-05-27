#pragma once

#include "graphics.hpp"
#include "timer.hpp"
#include "property_manager.hpp"
#include "generated/demo.types.hpp"
#include "effect.hpp"

namespace tano
{

  class Effect;
  typedef function<Effect*(const char*, u32)> EffectFactory;

  class DemoEngine
  {
  public:
    static DemoEngine &Instance();
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

    void WndProc(UINT message, WPARAM wParam, LPARAM lParam);
    sync_device* GetRocket() { return _rocket; }
    double GetRow() const;

    static bool IsInitialized() { return _instance != nullptr; }

    FileWatcher& GetFileWatcher() { return _fileWatcher; }
#if WITH_IMGUI
    PropertyManager& GetPropertyManager() { return _propertyManager; }
#endif

  private:

    DemoEngine();
    ~DemoEngine();
    static DemoEngine* _instance;

    void ReclassifyEffects();
    void KillEffects();

    void UpdateEffects();
    Effect* FindEffectByName(const string &name);
    bool ApplySettingsChange(const DemoSettings& settings);

    static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    deque<Effect*> _activeEffects;
    deque<Effect*> _inactiveEffects;
    deque<Effect*> _expiredEffects;
    vector<Effect*> _effects;
    int _cur_effect;
    TimeDuration _duration;
    TimeDuration _accumulated;

    Timer _timer;

    string _configFile;
    DemoSettings _settings;
    unordered_map<string, EffectFactory> _effectFactories;
    u32 _nextEffectId;

    FileWatcher _fileWatcher;
#if WITH_IMGUI
    PropertyManager _propertyManager;
#endif

#if WITH_MUSIC
    HSTREAM _stream = 0;
#endif

#if WITH_ROCKET
    sync_device* _rocket = nullptr;
#if !WITH_ROCKET_PLAYER
    bool _connected = false;
    TimeStamp _lastReconnect;
    static void RocketPauseCb(void* context, int flag);
    static void RocketSetRowCb(void* context, int row);
    static int RocketIsPlayingCb(void* context);
#endif
#endif
    UpdateState _initialState;
    Effect* _forceEffect = nullptr;
  };

#define DEMO_ENGINE DemoEngine::Instance()
#define ROCKET DemoEngine::Instance().GetRocket()
#define PROPERTIES DemoEngine::Instance().GetPropertyManager()

}
