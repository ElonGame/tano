#pragma once

#include "tano_math.hpp"

namespace tano
{
  struct Perlin2D
  {
    static void Init();
    static float Value(float x, float y);
  };
}
