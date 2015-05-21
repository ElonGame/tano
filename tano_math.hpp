#pragma once

namespace tano
{
  struct V3
  {
    V3() {}
    V3(float x, float y, float z) : x(x), y(y), z(z) {}
    V3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

    V3& operator+=(const V3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    V3& operator-=(const V3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    float x, y, z;
  };

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
}
