#pragma once
#include "demo_engine.hpp"

namespace tano
{
  struct AnimatedVar
  {
    AnimatedVar(const char* name);
    const sync_track* track;
  };

  template <typename T>
  struct AnimatedScalar
  {
    AnimatedScalar(const char* name) : v(name) {}
    operator T()
    {
      return (T)sync_get_val(v.track, DEMO_ENGINE.GetRow());
    }

    AnimatedVar v;
  };

  typedef AnimatedScalar<s32> AnimatedInt;
  typedef AnimatedScalar<float> AnimatedFloat;

  struct AnimatedBool
  {
    AnimatedBool(const char* name);
    operator bool();

    AnimatedVar v;
  };

  struct AnimatedVector3
  {
    AnimatedVector3(const char* name);
    operator Vector3();

    AnimatedVar x, y, z;
  };

  struct AnimatedColor
  {
    AnimatedColor(const char* name);
    operator Color();

    AnimatedVar r, g, b, a;
  };

}