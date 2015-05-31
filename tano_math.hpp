#pragma once

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
    V3(float x, float y, float z) : x(x), y(y), z(z) {}
    explicit V3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

    V3& operator+=(const V3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    V3& operator-=(const V3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    float x, y, z;
  };

  float Distance(const V3& a, const V3& b);
  float Length(const V3& a);
  float LengthSquared(const V3& a);
  V3 Normalize(const V3& v);
  V3 operator*(float s, const V3& v);
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
    return 1 / len * v;
  }

  inline V3 operator*(float s, const V3& v)
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

  inline Vector3 ClampVector(const Vector3& force, float maxLength)
  {
    float len = force.Length();
    if (len <= maxLength)
      return force;

    return maxLength / len * force;
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

}
