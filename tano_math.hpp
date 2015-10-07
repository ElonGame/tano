#pragma once

#define WITH_INVALID_CHECK 0

//using namespace DirectX;

namespace tano
{
  struct vec2
  {
    vec2() {}
    vec2(float x, float y) : x(x), y(y) {}

    float x, y;
  };

  vec2 Normalize(const vec2& v);
  float Dot(const vec2& a, const vec2& b);
  vec2 operator*(float s, const vec2& v);
  vec2 operator+(const vec2& a, const vec2& b);
  vec2 operator-(const vec2& a, const vec2& b);


  inline float Length(const vec2& a)
  {
    return sqrt(a.x*a.x + a.y*a.y);
  }

  inline float LengthSquared(const vec2& a)
  {
    return a.x*a.x + a.y*a.y;
  }

  inline vec2 Normalize(const vec2& v)
  {
    float len = Length(v);
    if (len == 0)
      return vec2(0,0);
    return 1 / len * v;
  }

  inline float Dot(const vec2& a, const vec2& b)
  {
    return a.x*b.x + a.y*b.y;
  }

  inline vec2 operator*(float s, const vec2& v)
  {
    return vec2(s*v.x, s*v.y);
  }

  inline vec2 operator+(const vec2& a, const vec2& b)
  {
    return vec2(a.x+b.x, a.y+b.y);
  }

  inline vec2 operator-(const vec2& a, const vec2& b)
  {
    return vec2(a.x - b.x, a.y - b.y);
  }

  struct vec3;
  float Dot(const vec3& a, const vec3& b);
  float Distance(const vec3& a, const vec3& b);
  float DistanceSquared(const vec3& a, const vec3& b);
  float Length(const vec3& a);
  float LengthSquared(const vec3& a);
  vec3 Normalize(const vec3& v);
  vec3 operator*(float s, const vec3& v);
  vec3 operator*=(float s, const vec3& v);
  vec3 operator*(const vec3& v, float s);
  vec3 operator-(const vec3& a, const vec3& b);
  vec3 operator+(const vec3& a, const vec3& b);

  struct vec3
  {
    static vec3 Zero;
    vec3() {}
#if WITH_INVALID_CHECK 
    vec3(float x, float y, float z) : x(x), y(y), z(z) { IsNan(); }
    explicit vec3(const Vector3& v) : x(v.x), y(v.y), z(v.z) { IsNan(); }
#else
    vec3(float x, float y, float z) : x(x), y(y), z(z) { }
#endif

    float& operator[](int idx) { return *(&x + idx); }
    float operator[] (int idx) const { return *(&x + idx); }
    vec3& operator+=(const vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    vec3& operator-=(const vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    vec3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }
    vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    bool operator==(const vec3& rhs) { return x == rhs.x && y == rhs.y && z == rhs.z; }
    void IsNan() { assert(!isnan(x)); assert(!isnan(y)); assert(!isnan(z)); }

    float Length() const { return tano::Length(*this); }
    float LengthSqr() const { return tano::LengthSquared(*this); }

    void Normalize()
    {
      *this = tano::Normalize(*this);
    }

    float x, y, z;
  };

  inline float Dot(const vec3& a, const vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

  inline float Distance(const vec3& a, const vec3& b)
  {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return sqrtf(dx*dx + dy*dy + dz*dz);
  }

  inline float DistanceSquared(const vec3& a, const vec3& b)
  {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return dx*dx + dy*dy + dz*dz;
  }

  inline float Length(const vec3& a)
  {
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
  }

  inline float LengthSquared(const vec3& a)
  {
    return a.x*a.x + a.y*a.y + a.z*a.z;
  }

  inline vec3 Normalize(const vec3& v)
  {
    float len = Length(v);
    return len > 0.0000001f ? 1 / len * v : vec3(0,0,0);
  }

  inline vec3 operator*(float s, const vec3& v)
  {
    return vec3(s*v.x, s*v.y, s*v.z);
  }

  inline vec3 operator*(const vec3& v, float s)
  {
    return vec3(s*v.x, s*v.y, s*v.z);
  }

  inline vec3 operator-(const vec3& a, const vec3& b)
  {
    return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  inline vec3 operator+(const vec3& a, const vec3& b)
  {
    return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
  }

  inline vec3 Cross(const vec3& a, const vec3& b)
  {
    return vec3(
      (a.y * b.z) - (a.z * b.y),
      (a.z * b.x) - (a.x * b.z),
      (a.x * b.y) - (a.y * b.x));
  }

  inline vec3 Min(const vec3& a, const vec3& b)
  {
    return vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
  }

  inline vec3 Max(const vec3& a, const vec3& b)
  {
    return vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
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
  inline vec3 ClampVector(const vec3& force, float maxLength)
  {
    float len = Length(force);
    if (len <= maxLength)
      return force;

    return maxLength / len * force;
  }

  /*inline Vector3 Normalize(const Vector3& v)
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
*/
  inline Vector3 ToVector3(const vec3& v)
  {
    return Vector3(v.x, v.y, v.z);
  }

  inline vec3 FromVector3(const Vector3& v)
  {
    return vec3(v.x, v.y, v.z);
  }

  struct vec4
  {
    vec4() {}
    vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    float x, y, z, w;
  };

  inline vec4 operator*(float s, const vec4& v)
  {
    return vec4(s * v.x, s * v.y, s * v.z, s * v.w);
  }

  inline vec4 operator+(const vec4& a, const vec4& b)
  {
    return vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
  }

  inline Vector4 ToVector4(const vec4& v)
  {
    return Vector4(v.x, v.y, v.z, v.w);
  }

  //------------------------------------------------------------------------------
  struct Spherical
  {
    float r;
    // phi is angle around x axis, ccw, starting at 0 at the x-axis
    // theta is angle around the z axis
    float phi, theta;
  };

  //------------------------------------------------------------------------------
  vec3 FromSpherical(float r, float phi, float theta);
  vec3 FromSpherical(const Spherical& s);
  Spherical ToSpherical(const vec3& v);

  //------------------------------------------------------------------------------
  inline int IntMod(int v, int m)
  {
    if (v < 0)
      v += m;
    return v % m;
  }

  //------------------------------------------------------------------------------
  struct Tri
  {
    int a, b, c;
  };

  //------------------------------------------------------------------------------
  inline Vector4 Expand(const Vector3& v, float x)
  {
    return Vector4(v.x, v.y, v.z, x);
  }

  //------------------------------------------------------------------------------
  vec3 PointOnHemisphere(const vec3& axis);
  vec3 RandomVector(float scaleX = 1, float scaleY = 1, float scaleZ = 1);

  //------------------------------------------------------------------------------
  struct CardinalSpline
  {
    void Create(const vec3* pts, int numPoints, float scale = 1.f);
    vec3 Interpolate(float t) const;
    vector<vec3> _controlPoints;
    float _scale;
  };

  //------------------------------------------------------------------------------
  inline vec3 Abs(const vec3& v)
  {
    return vec3(fabs(v.x), fabs(v.y), fabs(v.z));
  }

  //------------------------------------------------------------------------------
  // Returns a vector perpendicular to u, using the method by Hughes-Moller
  vec3 Perp(vec3 u);

}
