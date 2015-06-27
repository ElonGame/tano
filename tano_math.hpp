#pragma once

#define WITH_INVALID_CHECK 0

using namespace DirectX;

namespace tano
{
  struct V2
  {
    V2() {}
    V2(float x, float y) : x(x), y(y) {}

    float x, y;
  };

  V2 Normalize(const V2& v);
  float Dot(const V2& a, const V2& b);
  V2 operator*(float s, const V2& v);
  V2 operator+(const V2& a, const V2& b);
  V2 operator-(const V2& a, const V2& b);


  inline float Length(const V2& a)
  {
    return sqrt(a.x*a.x + a.y*a.y);
  }

  inline float LengthSquared(const V2& a)
  {
    return a.x*a.x + a.y*a.y;
  }

  inline V2 Normalize(const V2& v)
  {
    float len = Length(v);
    return 1 / len * v;
  }

  inline float Dot(const V2& a, const V2& b)
  {
    return a.x*b.x + a.y*b.y;
  }

  inline V2 operator*(float s, const V2& v)
  {
    return V2(s*v.x, s*v.y);
  }

  inline V2 operator+(const V2& a, const V2& b)
  {
    return V2(a.x+b.x, a.y+b.y);
  }

  inline V2 operator-(const V2& a, const V2& b)
  {
    return V2(a.x - b.x, a.y - b.y);
  }

  struct V3
  {
    V3() {}
#if WITH_INVALID_CHECK 
    V3(float x, float y, float z) : x(x), y(y), z(z) { IsNan(); }
    explicit V3(const Vector3& v) : x(v.x), y(v.y), z(v.z) { IsNan(); }
#else
    V3(float x, float y, float z) : x(x), y(y), z(z) { }
    explicit V3(const Vector3& v) : x(v.x), y(v.y), z(v.z) { }
#endif

    V3& operator+=(const V3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    V3& operator-=(const V3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    V3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    void IsNan() { assert(!isnan(x)); assert(!isnan(y)); assert(!isnan(z)); }

    float x, y, z;
  };

  float Distance(const V3& a, const V3& b);
  float Length(const V3& a);
  float LengthSquared(const V3& a);
  V3 Normalize(const V3& v);
  V3 operator*(float s, const V3& v);
  V3 operator*(const V3& v, float s);
  V3 operator-(const V3& a, const V3& b);
  V3 operator+(const V3& a, const V3& b);

  inline float Distance(const V3& a, const V3& b)
  {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return sqrtf(dx*dx + dy*dy + dz*dz);
  }

  inline float Length(const V3& a)
  {
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
  }

  inline float LengthSquared(const V3& a)
  {
    return a.x*a.x + a.y*a.y + a.z*a.z;
  }

  inline V3 Normalize(const V3& v)
  {
    float len = Length(v);
    return len > 0 ? 1 / len * v : v;
  }

  inline V3 operator*(float s, const V3& v)
  {
    return V3(s*v.x, s*v.y, s*v.z);
  }

  inline V3 operator*(const V3& v, float s)
  {
    return V3(s*v.x, s*v.y, s*v.z);
  }

  inline V3 operator-(const V3& a, const V3& b)
  {
    return V3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  inline V3 operator+(const V3& a, const V3& b)
  {
    return V3(a.x + b.x, a.y + b.y, a.z + b.z);
  }

  inline V3 Cross(const V3& a, const V3& b)
  {
    return V3(
      (a.y * b.z) - (a.z * b.y),
      (a.z * b.x) - (a.x * b.z),
      (a.x * b.y) - (a.y * b.x));
  }
/*
  inline Vector3 ClampVector(const Vector3& force, float maxLength)
  {
    float len = force.Length();
    if (len <= maxLength)
      return force;

    return maxLength / len * force;
  }
*/
  inline V3 ClampVector(const V3& force, float maxLength)
  {
    float len = Length(force);
    if (len <= maxLength)
      return force;

    return maxLength / len * force;
  }

  inline XMVECTOR XM_CALLCONV XMVector3ClampLengthMax(FXMVECTOR V, float maxLength)
  {
    XMVECTOR LengthMax = XMVectorReplicate(maxLength);

    assert((XMVectorGetY(LengthMax) == XMVectorGetX(LengthMax)) && (XMVectorGetZ(LengthMax) == XMVectorGetX(LengthMax)));
    assert(XMVector3GreaterOrEqual(LengthMax, XMVectorZero()));

    // ||r||^2
    XMVECTOR LengthSq = XMVector3LengthSq(V);

    const XMVECTOR Zero = XMVectorZero();

    // 1/||r||
    XMVECTOR RcpLength = XMVectorReciprocalSqrt(LengthSq);

    // 0xffffffff on equal, 0 otherwise
    XMVECTOR InfiniteLength = XMVectorEqualInt(LengthSq, g_XMInfinity.v);
    XMVECTOR ZeroLength = XMVectorEqual(LengthSq, Zero);

    // r/||r|| = r normalized
    XMVECTOR Normal = XMVectorMultiply(V, RcpLength);

    // ||r||^2 / ||r|| = ||r||
    XMVECTOR Length = XMVectorMultiply(LengthSq, RcpLength);

    // -0xff if not zero or inf
    XMVECTOR Select = XMVectorEqualInt(InfiniteLength, ZeroLength);
    Length = XMVectorSelect(LengthSq, Length, Select);
    Normal = XMVectorSelect(LengthSq, Normal, Select);

    XMVECTOR ControlMax = XMVectorGreater(Length, LengthMax);
    XMVECTOR ClampLength = XMVectorSelect(Length, LengthMax, ControlMax);

    XMVECTOR Result = XMVectorMultiply(Normal, ClampLength);

    // Preserve the original vector (with no precision loss) if the length is shorter than max
    XMVECTOR Control = XMVectorEqualInt(ControlMax, g_XMZero.v);
    Result = XMVectorSelect(Result, V, Control);

    return Result;
  }

  inline Vector3 Normalize(const Vector3& v)
  {
    float len = v.Length();
    if (len == 0.f)
      return Vector3(0, 0, 0);
    return 1 / len * v;
  }

  inline Vector3 Cross(const Vector3& a, const Vector3& b)
  {
    return Vector3(
      (a.y * b.z) - (a.z * b.y),
      (a.z * b.x) - (a.x * b.z),
      (a.x * b.y) - (a.y * b.x));
  }

  inline Vector3 ToVector3(const V3& v)
  {
    return Vector3(v.x, v.y, v.z);
  }

  struct V4
  {
    float x, y, z, w;
  };

}
