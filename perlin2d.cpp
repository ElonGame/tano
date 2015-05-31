#include "perlin2d.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
void Perlin2D::Init()
{
  // create random gradients
  for (int i = 0; i < SIZE; ++i)
    gradients[i] = Normalize(V2(randf(-1.f, 1.f), randf(-1.f, 1.f)));

  // create permutation table
  for (int i = 0; i < SIZE; ++i)
    permutation[i] = i;

  for (int i = 0; i < SIZE - 1; ++i)
    swap(permutation[i], permutation[randf(i + 1, SIZE - 1)]);
}

//------------------------------------------------------------------------------
inline float Perlin2D::Interp(float t)
{
  float t2 = t * t;
  float t3 = t2 * t;
  return 6 * t3 * t2 - 15 * t3 * t + 10 * t3;
}

//------------------------------------------------------------------------------
float Perlin2D::Value(float x, float y)
{
  return 0;
  int x0 = (int)floorf(x);
  int y0 = (int)floorf(y);

  int x1 = x0 + 1;
  int y1 = y0 + 1;

  // get gradients
  // g00  g01
  // g10  g11

  V2 g00 = gradients[permutation[(permutation[x0 & 0xff] + y0) & 0xff]];
  V2 g01 = gradients[permutation[(permutation[x1 & 0xff] + y0) & 0xff]];

  V2 g10 = gradients[permutation[(permutation[x0 & 0xff] + y1) & 0xff]];
  V2 g11 = gradients[permutation[(permutation[x1 & 0xff] + y1) & 0xff]];

  // get vectors from corners to center
  V2 p(x, y);
  V2 c00((float)x0, (float)y0);
  V2 c01((float)x1, (float)y0);
  V2 c10((float)x0, (float)y1);
  V2 c11((float)x1, (float)y1);

  V2 v00 = p - c00;
  V2 v01 = p - c01;
  V2 v10 = p - c10;
  V2 v11 = p - c11;

  // dot products
  float d00 = Dot(v00, g00);
  float d01 = Dot(v01, g01);
  float d10 = Dot(v10, g10);
  float d11 = Dot(v11, g11);

  // bilinear interpolation
  float dx = Interp(x - (float)x0);
  float dy = Interp(y - (float)y0);

  float xUpper = lerp(d00, d01, dx);
  float xLower = lerp(d10, d11, dx);

  float value = lerp(xUpper, xLower, dy);
  return value;
}

