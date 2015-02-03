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
TimeDuration DemoEngine::Pos() const
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
  _timer.SetCycle(duration);
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
  _propertyManager.Tick();

  TimeDuration delta, current;
  current = _timer.Elapsed(&delta);

  // check for any effects that have ended
  while (!_activeEffects.empty() && _activeEffects.front()->EndTime() <= current)
  {
    Effect* e = Transfer(_activeEffects, _expiredEffects);
    e->SetRunning(false);
  }

  // check if any effect start now
  unordered_set<Effect*> firstTimers;
  while (!_inactiveEffects.empty() && _inactiveEffects.front()->StartTime() <= current)
  {
    Effect* e = Transfer(_inactiveEffects, _activeEffects);
    firstTimers.insert(e);
    e->SetRunning(true);
  }

  bool paused = !_timer.IsRunning();

  // calc the number of ticks to step
  const int ticksPerSecond = 100;
  const float numTicks = TimeDurationToFloat(delta) * ticksPerSecond;
  const int intNumTicks = (int)numTicks;
  const float fracNumTicks = numTicks - intNumTicks;

  UpdateState curState;
  curState.globalTime = current;
  curState.delta = delta;
  curState.paused = paused;
  curState.frequency = ticksPerSecond;
  curState.numTicks = intNumTicks;
  curState.ticksFraction = fracNumTicks;

  UpdateState initialState;
  initialState.globalTime = current;
  initialState.localTime = TimeDuration::Seconds(0);
  initialState.delta = TimeDuration::Seconds(0);
  initialState.paused = paused;
  initialState.frequency = 100;
  initialState.numTicks = 1;
  initialState.ticksFraction = 0;

  // tick the active effects
  for (auto& effect : _activeEffects)
  {
    if (firstTimers.find(effect) != firstTimers.end())
    {
      effect->SetStartTime(current);
      effect->Update(initialState);
    }
    else
    {
      curState.localTime = current - effect->StartTime();
      effect->Update(curState);
    }

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

  ReclassifyEffects();

  return res;
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
