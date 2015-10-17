#include "demo_engine.hpp"
#include "base_effect.hpp"
#include "graphics.hpp"
#include "resource_manager.hpp"
#include "tano.hpp"
#include "init_sequence.hpp"
#include "generated/demo.parse.hpp"
#include "generated/demo.types.hpp"

using namespace tano;
using namespace bristol;

DemoEngine* tano::g_DemoEngine;

namespace
{
  // update frequeny in hz
  int UPDATE_FREQUENCY = 100;
  float UPDATE_INTERVAL = 1.0f / UPDATE_FREQUENCY;
  TimeDuration UPDATE_DELTA = TimeDuration::Microseconds((s64)1e6 / UPDATE_FREQUENCY);

  // These values are kinda arbitrary, and should probably be set to match
  // the music
  // const float bpm = 180.0f;
  // const float rpb = 8.0f;
  float ROWS_PER_SECOND = 24.0f; // bpm / 60.0f * rpb;
}

//------------------------------------------------------------------------------
double TimeDurationToFloat(TimeDuration t)
{
  return t.TotalMicroseconds() / 1e6;
}

//------------------------------------------------------------------------------
namespace
{
  BaseEffect* Transfer(deque<BaseEffect*>& src, deque<BaseEffect*>& dst)
  {
    BaseEffect* e = src.front();
    src.pop_front();
    dst.push_back(e);
    return e;
  }
}

//------------------------------------------------------------------------------
DemoEngine::DemoEngine() 
  : _cur_effect(0)
  , _duration(TimeDuration::Seconds(180))
  , _nextEffectId(1)
{
}

//------------------------------------------------------------------------------
DemoEngine::~DemoEngine()
{
  KillEffects();
  SeqDelete(&_effects);
  _activeEffects.clear();
  _inactiveEffects.clear();
  _expiredEffects.clear();

#if WITH_MUSIC
  BASS_StreamFree(_stream);
  BASS_Free();
#endif
}

//------------------------------------------------------------------------------
void DemoEngine::KillEffects()
{
}

//------------------------------------------------------------------------------
void DemoEngine::Create()
{
  assert(!g_DemoEngine);
  g_DemoEngine = new DemoEngine;
}

//------------------------------------------------------------------------------
bool DemoEngine::Start()
{
#if WITH_MUSIC
  BASS_Start();
  BASS_ChannelPlay(_stream, false);
  if (_settings.silent)
    BASS_ChannelSetAttribute(_stream, BASS_ATTRIB_VOL, 0);
#endif

  // Tick a few zero frames to make sure everything is allocated up front etc
  for (int i = 0; i < 3; ++i)
  {
    for (BaseEffect* effect : _effects)
    {
      if (!_forceEffect || effect == _forceEffect)
      {
        effect->Update(_initialState);
        effect->FixedUpdate(_initialFixedState);
        effect->Render();
      }
    }
  }

  _demoTimer.Start();
  _globalTimer.Start();

  if (_forceEffect)
  {
    SetPos(_forceEffect->StartTime());
  }

  return true;
}

//------------------------------------------------------------------------------
void DemoEngine::SetPaused(bool pause)
{
  if (pause)
  {
#if WITH_MUSIC
    if (_stream)
      BASS_ChannelPause(_stream);
#endif
    _demoTimer.Stop();
  }
  else
  {
#if WITH_MUSIC
    if (_stream)
      BASS_ChannelPlay(_stream, false);
#endif
    _demoTimer.Start();
  }
}

//------------------------------------------------------------------------------
bool DemoEngine::Paused() const
{
  return !_demoTimer.IsRunning();
}

//------------------------------------------------------------------------------
void DemoEngine::AdjustPos(const TimeDuration& delta)
{
  _demoTimer.Adjust(delta);
  ReclassifyEffects();
}

//------------------------------------------------------------------------------
void DemoEngine::SetPos(const TimeDuration& pos)
{
  TimeDuration clampedPos(pos);
  if (_forceEffect)
  {
    clampedPos.Clamp(_forceEffect->StartTime().GetTimestamp());
  }
  _demoTimer.SetElapsed(clampedPos);
#if WITH_MUSIC
  if (_stream)
  {
    QWORD pp = BASS_ChannelSeconds2Bytes(_stream, clampedPos.TotalMilliseconds() / 1e3);
    BASS_ChannelSetPosition(_stream, pp, BASS_POS_BYTE);
  }
#endif
  ReclassifyEffects();
}

//------------------------------------------------------------------------------
TimeDuration DemoEngine::Pos()
{
  return _demoTimer.Peek();
}

//------------------------------------------------------------------------------
TimeDuration DemoEngine::Duration() const
{
  return _duration;
}

//------------------------------------------------------------------------------
void DemoEngine::SetDuration(const TimeDuration& duration)
{
  _duration = duration;
}

//------------------------------------------------------------------------------
void DemoEngine::ReclassifyEffects()
{
  TimeDuration currentTime = _demoTimer.Elapsed(nullptr);

  sort(_effects.begin(), _effects.end(), [](const BaseEffect* a, const BaseEffect* b) 
  { 
    return a->StartTime() < b->StartTime(); 
  });

  _expiredEffects.clear();
  _inactiveEffects.clear();
  _activeEffects.clear();

  for (BaseEffect* effect : _effects)
  {
    bool started = currentTime >= effect->StartTime();
    bool ended = currentTime > effect->EndTime();
    effect->SetRunning(started && !ended);
    if (!started)
      _inactiveEffects.push_back(effect);
    else if (ended)
      _expiredEffects.push_back(effect);
    else
      _activeEffects.push_back(effect);
  }
}

//------------------------------------------------------------------------------
double DemoEngine::GetRow() const
{
#if WITH_MUSIC
  if (!_stream)
    return 0;

  QWORD pos = BASS_ChannelGetPosition(_stream, BASS_POS_BYTE);
  double time = BASS_ChannelBytes2Seconds(_stream, pos);
  double row = time * ROWS_PER_SECOND;
  return row;
#else
  return 0;
#endif
}

//------------------------------------------------------------------------------
bool DemoEngine::Tick()
{
  rmt_ScopedCPUSample(DemoEngine_Tick);

#if WITH_IMGUI
  _propertyManager.Tick();
#endif

  UpdateEffects();

  return true;
}

//------------------------------------------------------------------------------
void DemoEngine::UpdateEffects()
{
  rmt_ScopedCPUSample(DemoEngine_UpdateEffects);

  TimeDuration globalDelta;
  _globalTimer.Elapsed(&globalDelta);

  TimeDuration delta, current;
  current = _demoTimer.Elapsed(&delta);

  bool paused = !_demoTimer.IsRunning();

  // calc the number of ticks to step
  _updatedAcc += TimeDurationToFloat(delta);
  int numTicks = (int)(_updatedAcc / UPDATE_INTERVAL);
  _updatedAcc -= numTicks * UPDATE_INTERVAL;

  UpdateState curState;
  curState.globalTime = current;
  curState.delta = delta;
  curState.paused = paused;

  FixedUpdateState fixedState;
  fixedState.globalTime = current;
  fixedState.localTime = current;
  fixedState.delta = UPDATE_INTERVAL;
  fixedState.paused = paused;

  // do the common update for all effects (right now just updating the freefly camera)
  for (BaseEffect* effect : _effects)
  {
    effect->CommonUpdate(globalDelta.TotalSecondsAsFloat());
  }

  // If a force effect is set, just tick this guy
  if (_forceEffect)
  {
    curState.localTime = current - _forceEffect->StartTime();
    curState.globalTime = current;

    fixedState.localTime = current - _forceEffect->StartTime();
    if (_initForceEffect)
    {
      _initForceEffect = false;
      _forceEffect->Update(_initialState);
      _forceEffect->FixedUpdate(_initialFixedState);
    }
    else
    {
      _forceEffect->Update(curState);

      for (int i = 0; i < numTicks; ++i)
      {
        _forceEffect->FixedUpdate(fixedState);
      }
    }

    _forceEffect->Render();

    return;
  }

  // check for any effects that have ended
  while (!_activeEffects.empty() && _activeEffects.front()->EndTime() <= current)
  {
    BaseEffect* e = Transfer(_activeEffects, _expiredEffects);
    e->SetRunning(false);
  }

  // Update all active effects
  for (BaseEffect* effect : _activeEffects)
  {
    curState.localTime = current - effect->StartTime();
    effect->Update(curState);

    for (int i = 0; i < numTicks; ++i)
    {
      effect->FixedUpdate(fixedState);
    }
  }

  _initialState.globalTime = current;
  _initialState.paused = paused;

  // Tick all the effects that start now
  while (!_inactiveEffects.empty() && _inactiveEffects.front()->StartTime() <= current)
  {
    BaseEffect* e = Transfer(_inactiveEffects, _activeEffects);
    e->SetRunning(true);
    e->InitAnimatedParameters();
    e->Update(_initialState);
    e->FixedUpdate(_initialFixedState);
  }

  for (BaseEffect* effect : _activeEffects)
  {
    effect->Render();
  }
}

//------------------------------------------------------------------------------
void DemoEngine::Destroy()
{
  delete exch_null(g_DemoEngine);
}

//------------------------------------------------------------------------------
BaseEffect *DemoEngine::FindEffectByName(const string &name)
{
  auto it = find_if(_effects.begin(),
      _effects.end(),
      [&](const BaseEffect* e)
      {
        return e->InstanceName() == name;
      });
  return it == end(_effects) ? nullptr : *it;
}

//------------------------------------------------------------------------------
void DemoEngine::WndProc(UINT message, WPARAM wParam, LPARAM lParam)
{
}

//------------------------------------------------------------------------------
bool DemoEngine::ApplySettings(const DemoSettings& settings)
{
  BEGIN_INIT_SEQUENCE();

  bool hasForceEffect = false;
  for (const EffectSettings& e : settings.effects)
  {
    if (e.force)
    {
      VERIFY_FATAL(!hasForceEffect, "Only a single effect can be marked as force!");
      hasForceEffect = true;
    }
  }

  for (const EffectSettings& e : settings.effects)
  {
    // Look up the factory
    auto it = _effectFactories.find(e.factory);
    VERIFY_FATAL(it != _effectFactories.end(), "Unable to find factory: ", e.factory);

    // Create and init the effect
    EffectFactory factory = it->second;
    const char* configFile = e.settings.c_str();
    BaseEffect* effect = factory(e.name.c_str(), configFile, _nextEffectId++);
    _effects.push_back(effect);
#if WITH_IMGUI
    effect->RegisterParameters();
#endif

    TimeDuration start = TimeDuration::Milliseconds(e.start_time);
    TimeDuration end = TimeDuration::Milliseconds(e.end_time);
    effect->SetDuration(start, end);

    auto fnOnFileChanged = [this, effect](const string& filename)
    {
      vector<char> buf;
      INIT_FATAL_LOG(RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf), "Error loading config file: ", filename);
      return effect->OnConfigChanged(buf);
    };

    FileWatcherWin32::AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(configFile, true, fnOnFileChanged);

    // if using a force effect, only init that one
    if (!hasForceEffect || (hasForceEffect && e.force))
    {
      INIT_FATAL(effect->Init());
    }

    if (e.force)
    {
      _forceEffect = effect;
      _initForceEffect = true;
    }
  }

  // If a force effect is used, make his parameter set active
#if WITH_IMGUI
  if (_forceEffect)
  {
    PROPERTIES.SetActive(_forceEffect->InstanceName().c_str());
  }
#endif

  _settings = settings;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool DemoEngine::Init(const char* config, HINSTANCE instance)
{
  BEGIN_INIT_SEQUENCE();

  // Set up the initial effect state
  _initialState.localTime = TimeDuration::Seconds(0);
  _initialState.delta = TimeDuration::Seconds(0);

  _initialFixedState.globalTime = TimeDuration::Seconds(0);
  _initialFixedState.localTime = TimeDuration::Seconds(0);
  _initialFixedState.delta = UPDATE_INTERVAL;

  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(config, &buf));

  DemoSettings settings = ParseDemoSettings(InputBuffer(buf));
  INIT_FATAL(ApplySettings(settings));

#if WITH_MUSIC
  INIT(BASS_Init(-1, 44100, 0, 0, 0));
  INIT(_stream = BASS_StreamCreateFile(false, _settings.soundtrack.c_str(), 0, 0, BASS_STREAM_PRESCAN));
#endif

  ReclassifyEffects();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void DemoEngine::RegisterFactory(const string& type, const EffectFactory& factory)
{
  if (_effectFactories.count(type) > 0)
  {
    LOG_ERROR("Duplicate factory registration: ", type);
    return;
  }
  _effectFactories[type] = factory;
}

//------------------------------------------------------------------------------
LRESULT DemoEngine::WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  return DefWindowProc(hWnd, message, wParam, lParam);
}

