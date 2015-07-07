#include "tano_math.hpp"

using namespace bristol;

namespace tano
{
  //------------------------------------------------------------------------------
  V3 FromSpherical(float r, float phi, float theta)
  {
    float yProj = r * sinf(theta);
    return V3(yProj * cosf(phi), r * cosf(theta), yProj * sinf(phi));
  }

  //------------------------------------------------------------------------------
  V3 FromSpherical(const Spherical& s)
  {
    // phi is angle around x axis, ccw, starting at 0 at the x-axis
    // theta is angle around the z axis
    float yProj = s.r * sinf(s.theta);
    return V3(yProj * cosf(s.phi), s.r * cosf(s.theta), yProj * sinf(s.phi));
  }

  //------------------------------------------------------------------------------
  Spherical ToSpherical(const V3& v)
  {
    float r = Length(v);
    if (r == 0.f)
      return{ 0, 0, 0 };

    return{ r, atan2f(v.z, v.x), acosf(v.y / r) };
  }


  //------------------------------------------------------------------------------
  float ToneMap(float x, float c, float b, float w, float t, float s)
  {
    auto ToneMapK = [=]()
    {
      return ((1 - t)*(c - b)) / ((1 - s)*(w - c) + (1 - t)*(c - b));
    };

    auto ToneMapToe = [=](float k)
    {
      return (k*(1 - t)*(x - b)) / (c - (1 - t) * b - t * x);
    };

    auto ToneMapShoulder = [=](float k)
    {
      return k + ((1 - k)*(c - x)) / (s*x + (1 - s)*w - c);
    };

    float k = ToneMapK();
    return x < c ? ToneMapToe(k) : ToneMapShoulder(k);
  }

  //------------------------------------------------------------------------------
  V3 PointOnHemisphere(const V3& axis)
  {
    V3 tmp(randf(-1.f, +1.f), randf(-1.f, +1.f), randf(-1.f, +1.f));
    tmp = Normalize(tmp);
    if (Dot(axis, tmp) < 0)
      tmp = -1.f * tmp;

    return tmp;
  }


  //------------------------------------------------------------------------------
  V3 RandomVector(float scaleX, float scaleY, float scaleZ)
  {
    return V3(
      scaleX * randf(-1.f, +1.f),
      scaleY * randf(-1.f, +1.f),
      scaleZ * randf(-1.f, +1.f));
  }

}



