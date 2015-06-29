#include "tano_math.hpp"

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

}

