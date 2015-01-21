#include "effect.hpp"
#include "graphics.hpp"

using namespace tano;

//------------------------------------------------------------------------------
Effect::Effect(const string& name, u32 id)
  : _name(name)
  , _id(id)
  , _running(false)
  , _ctx(GRAPHICS.CreateDeferredContext())
  , _firstTick(true)
{
}

//------------------------------------------------------------------------------
Effect::~Effect()
{
  GRAPHICS.DestroyDeferredContext(_ctx);
}

//------------------------------------------------------------------------------
bool Effect::Show()
{
  return true;
}

//------------------------------------------------------------------------------
bool Effect::Hide()
{
  return true;
}

//------------------------------------------------------------------------------
bool Effect::Init(const char* configFile)
{
  return true;
}

//------------------------------------------------------------------------------
bool Effect::Reset()
{
  return true; 
}

//------------------------------------------------------------------------------
bool Effect::Update(const UpdateState& state)
{
  return true;
}

//------------------------------------------------------------------------------
bool Effect::Render()
{
  return true; 
}

//------------------------------------------------------------------------------
bool Effect::Close()
{
  return true; 
}

//------------------------------------------------------------------------------
void Effect::SetDuration(TimeDuration startTime, TimeDuration endTime)
{
  _startTime = startTime;
  _endTime = endTime;
}

//------------------------------------------------------------------------------
void Effect::SetStartTime(TimeDuration startTime)
{
  _startTime = startTime;
}

//------------------------------------------------------------------------------
bool Effect::Running() const
{
  return _running; 
}

//------------------------------------------------------------------------------
void Effect::SetRunning(bool b)
{ 
  _running = b; 
}
