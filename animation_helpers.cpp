#include "animation_helpers.hpp"
#include "demo_engine.hpp"

using namespace tano;

#if WITH_ROCKET
//------------------------------------------------------------------------------
AnimatedVar::AnimatedVar(const char* name) 
  : track(sync_get_track(ROCKET, name)) 
{
}

//------------------------------------------------------------------------------
AnimatedBool::AnimatedBool(const char* name) 
  : v(name) 
{
}

//------------------------------------------------------------------------------
AnimatedBool::operator bool()
{
  return sync_get_val(v.track, DEMO_ENGINE.GetRow()) > 0.0;
}

//------------------------------------------------------------------------------
AnimatedVector3::AnimatedVector3(const char* name)
  : x((string(name) + ".x").c_str())
  , y((string(name) + ".y").c_str())
  , z((string(name) + ".z").c_str())
{
}

//------------------------------------------------------------------------------
AnimatedVector3::operator Vector3()
{
  double row = DEMO_ENGINE.GetRow();
  return Vector3(
    (float)sync_get_val(x.track, row),
    (float)sync_get_val(y.track, row),
    (float)sync_get_val(z.track, row));
}

//------------------------------------------------------------------------------
AnimatedColor::AnimatedColor(const char* name)
  : r((string(name) + ".r").c_str())
  , g((string(name) + ".g").c_str())
  , b((string(name) + ".b").c_str())
  , a((string(name) + ".a").c_str())
{
}

//------------------------------------------------------------------------------
AnimatedColor::operator Color()
{
  double row = DEMO_ENGINE.GetRow();
  return Color(
    (float)sync_get_val(r.track, row),
    (float)sync_get_val(g.track, row),
    (float)sync_get_val(b.track, row),
    (float)sync_get_val(a.track, row));
}
#endif