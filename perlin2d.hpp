#pragma once

#include "tano_math.hpp"

namespace tano
{
  struct Perlin2D
  {
    void Init();
    float Value(float x, float y) const;

    float Interp(float t) const;

    static const int SIZE = 256;

    V2 gradients[SIZE];
    int permutation[SIZE];
  };
}
