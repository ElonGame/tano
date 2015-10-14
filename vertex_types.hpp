#pragma once
#include "tano_math.hpp"

// Some common vertex types collected in one place..

namespace tano
{
  struct TangentSpace
  {
    TangentSpace() {}
    TangentSpace(const vec3& tangent, const vec3& normal, const vec3& bitangent)
        : tangent(tangent), normal(normal), bitangent(bitangent)
    {
    }
    TangentSpace(
        float tx, float ty, float tz, float nx, float ny, float nz, float bx, float by, float bz)
        : tangent(tx, ty, tz), normal(nx, ny, nz), bitangent(bx, by, bz)
    {
    }
    vec3 tangent, normal, bitangent;
  };

  struct PosNormal
  {
    PosNormal() {}
    PosNormal(const vec3& pos, const vec3& normal) : pos(pos), normal(normal) {}
    PosNormal(float x, float y, float z, float nx, float ny, float nz)
        : pos(x, y, z), normal(nx, ny, nz)
    {
    }
    float operator[](int idx) const { return ((float*)&pos.x)[idx]; }
    float& operator[](int idx) { return ((float*)&pos.x)[idx]; }
    vec3 pos;
    vec3 normal;
  };

  struct PosNormalColor
  {
    PosNormalColor() {}
    PosNormalColor(const vec3& pos, const vec3& normal, const vec4& col)
        : pos(pos), normal(normal), col(col)
    {
    }
    PosNormalColor(
        float x, float y, float z, float nx, float ny, float nz, float r, float g, float b, float a)
        : pos(x, y, z), normal(nx, ny, nz), col(r, g, b, a)
    {
    }
    float operator[](int idx) const { return ((float*)&pos.x)[idx]; }
    float& operator[](int idx) { return ((float*)&pos.x)[idx]; }
    vec3 pos;
    vec3 normal;
    vec4 col;
  };

  struct PosTangentSpace
  {
    PosTangentSpace() {}
    PosTangentSpace(const vec3& pos, const vec3& normal, const vec3& tangent, const vec3& binormal)
        : pos(pos), normal(normal), tangent(tangent), binormal(binormal)
    {
    }
    vec3 pos;
    vec3 normal, tangent, binormal;
  };

  struct PosTangentSpaceTex
  {
    PosTangentSpaceTex() {}
    PosTangentSpaceTex(const vec3& pos,
        const vec3& normal,
        const vec3& tangent,
        const vec3& binormal,
        const vec2& tex)
        : pos(pos), normal(normal), tangent(tangent), binormal(binormal), tex(tex)
    {
    }
    vec3 pos;
    vec3 normal, tangent, binormal;
    vec2 tex;
  };

  // bitangent = cross(normal, tangent)
  struct PosTangentSpace2
  {
    PosTangentSpace2() {}
    PosTangentSpace2(const vec3& pos, const vec2& tex, const vec3& tangent, const vec3& normal)
        : pos(pos), tex(tex), tangent(tangent), normal(normal)
    {
    }
    vec3 pos;
    vec2 tex;
    vec3 tangent, normal;
  };

  struct Pos2Tex
  {
    Pos2Tex() {}
    Pos2Tex(const vec2& pos, const vec2& tex) : pos(pos), tex(tex) {}
    Pos2Tex(float x, float y, float u, float v) : pos(x, y), tex(u, v) {}
    vec2 pos;
    vec2 tex;
  };

  struct PosTex
  {
    PosTex() {}
    PosTex(const vec3& pos, const vec2& tex) : pos(pos), tex(tex) {}
    PosTex(float x, float y, float z, float u, float v) : pos(x, y, z), tex(u, v) {}
    vec3 pos;
    vec2 tex;
  };

  struct Pos4Tex
  {
    Pos4Tex() {}
    Pos4Tex(const vec4& pos, const vec2& tex) : pos(pos), tex(tex) {}
    vec4 pos;
    vec2 tex;
  };

  struct PosTex3
  {
    PosTex3() {}
    PosTex3(const vec3& pos, const vec3& tex) : pos(pos), tex(tex) {}
    PosTex3(float x, float y, float z, float u, float v, float w) : pos(x, y, z), tex(u, v, w) {}
    vec3 pos;
    vec3 tex;
  };

  struct PosColTex
  {
    PosColTex() {}
    PosColTex(const vec3& pos, const vec4& col, const vec2& tex) : pos(pos), col(col), tex(tex) {}
    PosColTex(float x, float y, float z, float r, float g, float b, float a, float u, float v)
        : pos(x, y, z), col(r, g, b, a), tex(u, v)
    {
    }
    vec3 pos;
    vec4 col;
    vec2 tex;
  };

  struct PosCol
  {
    PosCol() {}
    PosCol(float x, float y, float z, float r, float g, float b, float a)
        : pos(x, y, z), col(r, g, b, a)
    {
    }
    PosCol(float x, float y, float z, const vec4& col) : pos(x, y, z), col(col) {}
    PosCol(const vec3& pos, const vec4& col) : pos(pos), col(col) {}
    PosCol(const vec2& pos, float z, const vec4& col) : pos(pos.x, pos.y, z), col(col) {}
    PosCol(const vec3& pos) : pos(pos) {}
    vec3 pos;
    vec4 col;
  };

  struct PosNormalTex
  {
    PosNormalTex() {}
    PosNormalTex(const vec3& pos, const vec3& normal, const vec2& tex)
        : pos(pos), normal(normal), tex(tex)
    {
    }
    vec3 pos;
    vec3 normal;
    vec2 tex;
  };
}
