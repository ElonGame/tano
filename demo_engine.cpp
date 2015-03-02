#include "demo_engine.hpp"
#include "effect.hpp"
#include "graphics.hpp"
#include "resource_manager.hpp"
#include "tano.hpp"
#include "init_sequence.hpp"
#include "generated/input_buffer.hpp"
#include "generated/demo.parse.hpp"
#include "generated/demo.types.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  // update frequeny in hz
  u32 UPDATE_FREQUENCY = 100;
  TimeDuration UPDATE_DELTA = TimeDuration::Microseconds((s64)1e6 / UPDATE_FREQUENCY);

  // These values are kinda arbitrary, and should probably be set to match
  // the music
  // const float bpm = 180.0f;
  // const float rpb = 8.0f;
  float ROWS_PER_SECOND = 24.0f; // bpm / 60.0f * rpb;
}

//------------------------------------------------------------------------------
DemoEngine *DemoEngine::_instance = NULL;

//------------------------------------------------------------------------------
float TimeDurationToFloat(TimeDuration t)
{
  return t.TotalMicroseconds() / 1e6f;
}

//------------------------------------------------------------------------------
namespace
{
  Effect* Transfer(deque<Effect*>& src, deque<Effect*>& dst)
  {
    Effect* e = src.front();
    src.pop_front();
    dst.push_back(e);
    return e;
  }
}

//------------------------------------------------------------------------------
DemoEngine& DemoEngine::Instance()
{
  assert(_instance);
  return *_instance;
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

#if !WITH_ROCKET_PLAYER
  sync_save_tracks(_rocket);
#endif
  sync_destroy_device(_rocket);

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
bool DemoEngine::Create()
{
  assert(!_instance);
  _instance = new DemoEngine;
  return true;
}

//------------------------------------------------------------------------------
bool DemoEngine::Start()
{
  _timer.Start();
#if WITH_MUSIC
  BASS_Start();
  BASS_ChannelPlay(_stream, false);
  if (_settings.silent)
    BASS_ChannelSetAttribute(_stream, BASS_ATTRIB_VOL, 0);
#endif

  return true;
}

//------------------------------------------------------------------------------
void DemoEngine::SetPaused(bool pause)
{
  if (pause)
  {
    if (_stream)
      BASS_ChannelPause(_stream);
    _timer.Stop();
  }
  else
  {
    if (_stream)
      BASS_ChannelPlay(_stream, false);
    _timer.Start();
  }
}

//------------------------------------------------------------------------------
bool DemoEngine::Paused() const
{
  return !_timer.IsRunning();
}

//------------------------------------------------------------------------------
void DemoEngine::AdjustPos(const TimeDuration& delta)
{
  _timer.Adjust(delta);
  ReclassifyEffects();
}

//------------------------------------------------------------------------------
void DemoEngine::SetPos(const TimeDuration& pos)
{
  _timer.SetElapsed(pos);

  if (_stream)
  {
    QWORD pp = BASS_ChannelSeconds2Bytes(_stream, pos.TotalMilliseconds() / 1e3);
    BASS_ChannelSetPosition(_stream, pp, BASS_POS_BYTE);
  }

  ReclassifyEffects();
}

//------------------------------------------------------------------------------
TimeDuration DemoEngine::Pos()
{
  return _timer.Peek();
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
  TimeDuration currentTime = _timer.Elapsed(nullptr);

  sort(_effects.begin(), _effects.end(), [](const Effect* a, const Effect* b) { return a->StartTime() < b->StartTime(); });

  _expiredEffects.clear();
  _inactiveEffects.clear();
  _activeEffects.clear();

  for (auto& effect : _effects)
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
  if (!_stream)
    return 0;

  QWORD pos = BASS_ChannelGetPosition(_stream, BASS_POS_BYTE);
  double time = BASS_ChannelBytes2Seconds(_stream, pos);
  double row = time * ROWS_PER_SECOND;
  return row;
}

//------------------------------------------------------------------------------
bool DemoEngine::Tick()
{
#if !WITH_ROCKET_PLAYER
  static struct sync_cb RocketCb =
  {
    RocketPauseCb,
    RocketSetRowCb,
    RocketIsPlayingCb,
  };
#endif

  rmt_ScopedCPUSample(DemoEngine_Tick);

  _fileWatcher.Tick();
#if WITH_IMGUI
  _propertyManager.Tick();
#endif

#if WITH_ROCKET
  if (_stream)
  {
    double row = GetRow();

#if !WITH_ROCKET_PLAYER
    if (!_connected)
    {
      TimeStamp now = TimeStamp::Now();
      if (now - _lastReconnect > TimeDuration::Seconds(5))
      {
        _connected = sync_connect(_rocket, "localhost", SYNC_DEFAULT_PORT) == 0;
        _lastReconnect = now;
      }
    }
    else
    {
      if (sync_update(_rocket, (int)(row), &RocketCb, this))
      {
        _connected = false;
      }
    }
#endif
  }
#endif

  UpdateEffects();

  return true;
}

//------------------------------------------------------------------------------
void DemoEngine::UpdateEffects()
{
  TimeDuration delta, current;
  current = _timer.Elapsed(&delta);

  bool paused = !_timer.IsRunning();

  // calc the number of ticks to step
  float numTicks = TimeDurationToFloat(delta) * UPDATE_FREQUENCY;
  int intNumTicks = (int)numTicks;

  UpdateState curState;
  curState.globalTime = current;
  curState.delta = delta;
  curState.paused = paused;
  curState.frequency = UPDATE_FREQUENCY;
  curState.numTicks = intNumTicks;
  curState.ticksFraction = numTicks - intNumTicks;


  // If a force effect is set, just tick this guy
  if (_forceEffect)
  {
    curState.localTime = current;
    _forceEffect->Update(curState);
    _forceEffect->Render();
    return;
  }

  // check for any effects that have ended
  while (!_activeEffects.empty() && _activeEffects.front()->EndTime() <= current)
  {
    Effect* e = Transfer(_activeEffects, _expiredEffects);
    e->SetRunning(false);
  }

  // tick the active effects
  for (auto& effect : _activeEffects)
  {
    curState.localTime = current - effect->StartTime();
    effect->Update(curState);
  }

  _initialState.globalTime = current;
  _initialState.paused = paused;

  // Tick all the effects that start now
  while (!_inactiveEffects.empty() && _inactiveEffects.front()->StartTime() <= current)
  {
    Effect* e = Transfer(_inactiveEffects, _activeEffects);
    e->SetRunning(true);
    e->InitAnimatedParameters();
    e->Update(_initialState);
  }

  for (auto& effect : _activeEffects)
  {
    effect->Render();
  }
}

//------------------------------------------------------------------------------
bool DemoEngine::Destroy()
{
  delete exch_null(_instance);
  return true;
}

//------------------------------------------------------------------------------
Effect *DemoEngine::FindEffectByName(const string &name)
{
  auto it = find_if(_effects.begin(), _effects.end(),
      [&](const Effect *e) { return e->Name() == name; });
  return it == end(_effects) ? nullptr : *it;
}

//------------------------------------------------------------------------------
void DemoEngine::WndProc(UINT message, WPARAM wParam, LPARAM lParam)
{
}

//------------------------------------------------------------------------------
bool DemoEngine::ApplySettingsChange(const DemoSettings& settings)
{
  BEGIN_INIT_SEQUENCE();

  for (auto& e : settings.effects)
  {
    // Look up the factory
    auto it = _effectFactories.find(e.factory);
    if (it == _effectFactories.end())
    {
      INJECT_ERROR(ToString("Unable to find factory: %s", e.factory.c_str()).c_str());
      continue;
    }

    // Create and init the effect
    EffectFactory factory = it->second;
    Effect* effect = factory(e.name.c_str(), _nextEffectId++);
    _effects.push_back(effect);

    TimeDuration start = TimeDuration::Milliseconds(e.start_time);
    TimeDuration end = TimeDuration::Milliseconds(e.end_time);
    effect->SetDuration(start, end);
    INIT(effect->Init(e.settings.c_str()));

    if (e.force)
    {
      if (_forceEffect)
      {
        INJECT_ERROR("Only a single effect can be marked as force!");
        continue;
      }
      _forceEffect = effect;
    }
  }

  _settings = settings;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool DemoEngine::Init(const char* config, HINSTANCE instance)
{
  BEGIN_INIT_SEQUENCE();
  bool res = true;

#if WITH_ROCKET 
  INIT(_rocket = sync_create_device("config/sync/"));
#if !WITH_ROCKET_PLAYER
  _connected = sync_connect(_rocket, "localhost", SYNC_DEFAULT_PORT) == 0;
  _lastReconnect = TimeStamp::Now();
#endif
#endif

  _fileWatcher.AddFileWatch(config, nullptr, true, &res, [this](const string& filename, void* token)
  {
    vector<char> buf;
    if (!RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf))
      return false;

    DemoSettings settings;
    if (!ParseDemoSettings(InputBuffer(buf), &settings))
      return false;

    if (!ApplySettingsChange(settings))
      return false;

    return true;
  });

#if WITH_MUSIC
  INIT(BASS_Init(-1, 44100, 0, 0, 0));
  INIT(_stream = BASS_StreamCreateFile(false, _settings.soundtrack.c_str(), 0, 0, BASS_STREAM_PRESCAN));
#endif

  // Set up the initial effect state
  _initialState.localTime = TimeDuration::Seconds(0);
  _initialState.delta = TimeDuration::Seconds(0);
  _initialState.frequency = UPDATE_FREQUENCY;
  _initialState.numTicks = 1;
  _initialState.ticksFraction = 0;

  ReclassifyEffects();

  INIT(res);
  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void DemoEngine::RegisterFactory(const string& type, const EffectFactory& factory)
{
  _effectFactories[type] = factory;
}

//------------------------------------------------------------------------------
LRESULT DemoEngine::WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  return DefWindowProc(hWnd, message, wParam, lParam);
}

#if !WITH_ROCKET_PLAYER
//------------------------------------------------------------------------------
void DemoEngine::RocketPauseCb(void* context, int flag)
{
  DemoEngine* self = (DemoEngine*)context;
  if (HSTREAM h = self->_stream)
  {
    if (flag)
    {
      BASS_ChannelPause(h);
      self->_timer.Stop();
    }
    else
    {
      BASS_ChannelPlay(h, false);
      self->_timer.Start();
    }
  }
}

//------------------------------------------------------------------------------
void DemoEngine::RocketSetRowCb(void* context, int row)
{
  DemoEngine* self = (DemoEngine*)context;

  if (HSTREAM h = self->_stream)
  {
    double time = row / ROWS_PER_SECOND;
    self->_timer.SetElapsed(TimeDuration::Microseconds((s64)(1e6 * time)));
    QWORD pos = BASS_ChannelSeconds2Bytes(h, time);
    BASS_ChannelSetPosition(h, pos, BASS_POS_BYTE);
  }
}

//------------------------------------------------------------------------------
int DemoEngine::RocketIsPlayingCb(void* context)
{
  if (HSTREAM h = ((DemoEngine*)context)->_stream)
  {
    return BASS_ChannelIsActive(h) == BASS_ACTIVE_PLAYING;
  }
  return 0;
}
#endif
