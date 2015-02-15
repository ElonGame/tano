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
  const struct sync_track *clear_r, *clear_g, *clear_b;

  // update frequeny in hz
  u32 UPDATE_FREQUENCY = 100;
  TimeDuration UPDATE_DELTA = TimeDuration::Microseconds((s64)1e6 / UPDATE_FREQUENCY);

  // these values come from the rocket editor
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

#if WITH_MUSIC
  BASS_StreamFree(_stream);
  BASS_Free();
//   _sound->release();
//   _system->close();
//   _system->release();
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
#endif

  return true;
}

//------------------------------------------------------------------------------
void DemoEngine::SetPaused(bool pause)
{
  if (pause)
    _timer.Stop();
  else
    _timer.Start();
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
bool DemoEngine::Tick()
{
  rmt_ScopedCPUSample(DemoEngine_Tick);

  _fileWatcher.Tick();
#if WITH_IMGUI
  _propertyManager.Tick();
#endif

#if WITH_MUSIC
//  _system->update();
#endif

#if WITH_ROCKET
  if (_stream)
  {
    QWORD pos = BASS_ChannelGetPosition(_stream, BASS_POS_BYTE);
    double time = BASS_ChannelBytes2Seconds(_stream, pos);
    double row = time * ROWS_PER_SECOND;
    static struct sync_cb RocketCb =
    {
      RocketPauseCb,
      RocketSetRowCb,
      RocketIsPlayingCb,
    };

    if (sync_update(_rocket, (int)(row), &RocketCb, this))
      sync_connect(_rocket, "localhost", SYNC_DEFAULT_PORT);

    double r = sync_get_val(clear_r, row);
    double g = sync_get_val(clear_g, row);
    double b = sync_get_val(clear_b, row);
  }
#endif

  TimeDuration delta, current;
  current = _timer.Elapsed(&delta);

  // check for any effects that have ended
  while (!_activeEffects.empty() && _activeEffects.front()->EndTime() <= current)
  {
    Effect* e = Transfer(_activeEffects, _expiredEffects);
    e->SetRunning(false);
  }

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

  // tick the active effects
  for (auto& effect : _activeEffects)
  {
    curState.localTime = current - effect->StartTime();
    effect->Update(curState);
  }

  // check if any effect start now
  UpdateState initialState;
  initialState.globalTime = current;
  initialState.localTime = TimeDuration::Seconds(0);
  initialState.delta = TimeDuration::Seconds(0);
  initialState.paused = paused;
  initialState.frequency = UPDATE_FREQUENCY;
  initialState.numTicks = 1;
  initialState.ticksFraction = 0;

  // Tick all the first timers
  while (!_inactiveEffects.empty() && _inactiveEffects.front()->StartTime() <= current)
  {
    Effect* e = Transfer(_inactiveEffects, _activeEffects);
    e->SetRunning(true);
    e->Update(initialState);
  }

  for (auto& effect : _activeEffects)
  {
    effect->Render();
  }

  return true;
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
  }

  // If we get this far, apply the new settings
  // Ok, we can't actually roll out of anything bad..
  _settings = settings;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool DemoEngine::Init(const char* config, HINSTANCE instance)
{
  BEGIN_INIT_SEQUENCE();
  bool res = true;

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

#if WITH_ROCKET
  INIT(_rocket = sync_create_device("sync"));
  INIT(sync_connect(_rocket, "localhost", SYNC_DEFAULT_PORT) == 0);

  clear_r = sync_get_track(_rocket, "clear.r");
  clear_g = sync_get_track(_rocket, "clear.g");
  clear_b = sync_get_track(_rocket, "clear.b");
#endif

#if WITH_MUSIC
  INIT(BASS_Init(-1, 44100, 0, 0, 0));
  INIT(_stream = BASS_StreamCreateFile(false, _settings.soundtrack.c_str(), 0, 0, BASS_STREAM_PRESCAN));
#endif

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

//------------------------------------------------------------------------------
void DemoEngine::RocketPauseCb(void* context, int flag)
{
  if (HSTREAM h = ((DemoEngine*)context)->_stream)
  {
    if (flag)
      BASS_ChannelPause(h);
    else
      BASS_ChannelPlay(h, false);
  }
}

//------------------------------------------------------------------------------
void DemoEngine::RocketSetRowCb(void* context, int row)
{
  if (HSTREAM h = ((DemoEngine*)context)->_stream)
  {
    QWORD pos = BASS_ChannelSeconds2Bytes(h, row / ROWS_PER_SECOND);
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
