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

  //------------------------------------------------------------------------------
  V3 CardinalSplineBlend(const V3& p0, const V3& p1, const V3& p2, const V3& p3, float s)
  {
    float a = 0.5f;

    V3 t1 = a*(p2 - p0);
    V3 t2 = a*(p3 - p1);

    // P = h1 * P1 + h2 * P2 + h3 * T1 + h4 * T2;
    float s2 = s*s;
    float s3 = s2*s;
    float h1 = +2 * s3 - 3 * s2 + 1;
    float h2 = -2 * s3 + 3 * s2;
    float h3 = s3 - 2 * s2 + s;
    float h4 = s3 - s2;

    return h1 * p1 + h2 * p2 + h3 * t1 + h4 * t2;
  }

  //------------------------------------------------------------------------------
  void CardinalSpline::Create(const V3* pts, int numPoints)
  {
    int numSteps = 50;
    float sInc = 1.0f / numSteps;
    float a = 0.5f;

    controlPoints.resize(numPoints);
    copy(pts, pts + numPoints, controlPoints.begin());
    spline.reserve(numPoints * numSteps);

    for (int i = 0; i < numPoints - 1; ++i)
    {
      V3 p0 = controlPoints[max(0, i - 1)];
      V3 p1 = controlPoints[i];
      V3 p2 = controlPoints[i + 1];
      V3 p3 = controlPoints[min(numPoints - 1, i + 2)];

      V3 t1 = a*(p2 - p0);
      V3 t2 = a*(p3 - p1);

      float s = 0;
      for (int j = 0; j < numSteps; ++j)
      {
        // P = h1 * P1 + h2 * P2 + h3 * T1 + h4 * T2;
        float s2 = s*s;
        float s3 = s2*s;
        float h1 = +2 * s3 - 3 * s2 + 1;
        float h2 = -2 * s3 + 3 * s2;
        float h3 = s3 - 2 * s2 + s;
        float h4 = s3 - s2;

        spline.push_back(h1 * p1 + h2 * p2 + h3 * t1 + h4 * t2);
        s += sInc;
      }
    }
  }

  //------------------------------------------------------------------------------
  V3 CardinalSpline::Interpolate()
  {
    int numPts = (int)controlPoints.size();
    int intTime = (int)curTime;
    float fracTime = curTime - intTime;
    int idx = intTime % numPts;

    V3 p0 = controlPoints[max(0, idx - 1)];
    V3 p1 = controlPoints[idx];
    V3 p2 = controlPoints[idx + 1];
    V3 p3 = controlPoints[min(numPts - 1, idx + 2)];

    return CardinalSplineBlend(p0, p1, p2, p3, fracTime);
  }

  //------------------------------------------------------------------------------
  void CardinalSpline::Update(float dt)
  {
    curTime += dt * speed;
  }

  //------------------------------------------------------------------------------
  void CardinalSpline2::Create(const V3* pts, int numPoints, float scale)
  {
    assert(scale != 0);
    _scale = scale;
    _controlPoints.resize(numPoints);
    copy(pts, pts + numPoints, _controlPoints.begin());
  }

  //------------------------------------------------------------------------------
  V3 CardinalSpline2::Interpolate(float t) const
  {
    int numPoints = (int)_controlPoints.size();
    t /= _scale;
    int i = (int)t;

    int m = numPoints - 1;
    V3 p0 = _controlPoints[min(m, max(0, i - 1))];
    V3 p1 = _controlPoints[min(m, max(0, i + 0))];
    V3 p2 = _controlPoints[min(m, max(0, i + 1))];
    V3 p3 = _controlPoints[min(m, max(0, i + 2))];

    float s = t - (float)i;
    return CardinalSplineBlend(p0, p1, p2, p3, s);

  }

  //------------------------------------------------------------------------------
  // Returns a vector perpendicular to u, using the method by Hughes-Moller
  V3 Perp(V3 u)
  {
    V3 a = Abs(u);
    V3 v;
    if (a.x <= a.y && a.x <= a.z)
      v = V3(0, -u.z, u.y);
    else if (a.y <= a.x && a.y <= a.z)
      v = V3(-u.z, 0, u.x);
    else
      v = V3(-u.y, u.x, 0);

    return Normalize(v);
  }

}
