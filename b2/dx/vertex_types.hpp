#pragma once
#include "DirectXTK/SimpleMath.h"

// Some common vertex types collected in one place..
namespace bristol
{
  struct TangentSpace
  {
    TangentSpace() {}
    TangentSpace(const DirectX::SimpleMath::Vector3 &tangent, const DirectX::SimpleMath::Vector3& normal, const DirectX::SimpleMath::Vector3 &bitangent)
      : tangent(tangent), normal(normal), bitangent(bitangent) {}
    TangentSpace(float tx, float ty, float tz, float nx, float ny, float nz, float bx, float by, float bz) 
      : tangent(tx, ty, tz), normal(nx, ny, nz), bitangent(bx, by, bz) {}
    DirectX::SimpleMath::Vector3 tangent, normal, bitangent;
  };


  struct PosNormal
  {
    PosNormal() {}
    PosNormal(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& normal) : pos(pos), normal(normal) {}
    PosNormal(float x, float y, float z, float nx, float ny, float nz) : pos(x, y, z), normal(nx, ny, nz) {}
    float operator[](int idx) const { return ((float*)&pos.x)[idx]; }
    float& operator[](int idx) { return ((float*)&pos.x)[idx]; }
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 normal;
  };

  struct PosNormalColor
  {
    PosNormalColor() {}
    PosNormalColor(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& normal, const DirectX::SimpleMath::Color& col) : pos(pos), normal(normal), col(col) {}
    PosNormalColor(float x, float y, float z, float nx, float ny, float nz, float r, float g, float b, float a) : pos(x, y, z), normal(nx, ny, nz), col(r, g, b, a) {}
    float operator[](int idx) const { return ((float*)&pos.x)[idx]; }
    float& operator[](int idx) { return ((float*)&pos.x)[idx]; }
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 normal;
    DirectX::SimpleMath::Color col;
  };

  struct PosTangentSpace
  {
    PosTangentSpace() {}
    PosTangentSpace(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& normal, const DirectX::SimpleMath::Vector3 &tangent, const DirectX::SimpleMath::Vector3 &binormal)
      : pos(pos), normal(normal), tangent(tangent), binormal(binormal){}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 normal, tangent, binormal;
  };

  struct PosTangentSpaceTex
  {
    PosTangentSpaceTex() {}
    PosTangentSpaceTex(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& normal, const DirectX::SimpleMath::Vector3 &tangent, const DirectX::SimpleMath::Vector3 &binormal, const DirectX::SimpleMath::Vector2 &tex)
      : pos(pos), normal(normal), tangent(tangent), binormal(binormal), tex(tex) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 normal, tangent, binormal;
    DirectX::SimpleMath::Vector2 tex;
  };

  // bitangent = cross(normal, tangent)
  struct PosTangentSpace2
  {
    PosTangentSpace2() {}
    PosTangentSpace2(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector2 &tex, const DirectX::SimpleMath::Vector3 &tangent, const DirectX::SimpleMath::Vector3& normal) 
      : pos(pos), tex(tex), tangent(tangent), normal(normal) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector2 tex;
    DirectX::SimpleMath::Vector3 tangent, normal;
  };

  struct Pos2Tex
  {
    Pos2Tex() {}
    Pos2Tex(const DirectX::SimpleMath::Vector2& pos, const DirectX::SimpleMath::Vector2& tex) : pos(pos), tex(tex) {}
    Pos2Tex(float x, float y, float u, float v) : pos(x, y), tex(u, v) {}
    DirectX::SimpleMath::Vector2 pos;
    DirectX::SimpleMath::Vector2 tex;
  };

  struct PosTex
  {
    PosTex() {}
    PosTex(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector2& tex) : pos(pos), tex(tex) {}
    PosTex(float x, float y, float z, float u, float v) : pos(x, y, z), tex(u, v) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector2 tex;
  };

  struct PosTex3
  {
    PosTex3() {}
    PosTex3(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& tex) : pos(pos), tex(tex) {}
    PosTex3(float x, float y, float z, float u, float v, float w) : pos(x, y, z), tex(u, v, w) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 tex;
  };

  struct PosColTex
  {
    PosColTex() {}
    PosColTex(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector4& col, const DirectX::SimpleMath::Vector2& tex) : pos(pos), col(col), tex(tex) {}
    PosColTex(float x, float y, float z, float r, float g, float b, float a, float u, float v) : pos(x, y, z), col(r, g, b, a), tex(u, v) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector4 col;
    DirectX::SimpleMath::Vector2 tex;
  };

  struct Pos4Tex
  {
    Pos4Tex() {}
    Pos4Tex(const DirectX::SimpleMath::Vector4 &pos, const DirectX::SimpleMath::Vector2& tex) : pos(pos), tex(tex) {}
    Pos4Tex(float x, float y, float z, float w, float u, float v) : pos(x, y, z, w), tex(u, v) {}
    DirectX::SimpleMath::Vector4 pos;
    DirectX::SimpleMath::Vector2 tex;
  };

  struct PosCol
  {
    PosCol() {}
    PosCol(float x, float y, float z, float r, float g, float b, float a) : pos(x, y, z), col(r, g, b, a) {}
    PosCol(float x, float y, float z, const DirectX::SimpleMath::Vector4& col) : pos(x, y, z), col(col) {}
    PosCol(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Color& col) : pos(pos), col(col) {}
    PosCol(const DirectX::SimpleMath::Vector2& pos, float z, const DirectX::SimpleMath::Color& col) : pos(pos.x, pos.y, z), col(col) {}
    PosCol(const DirectX::SimpleMath::Vector3& pos) : pos(pos) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Color col;
  };

  struct PosNormalTex
  {
    PosNormalTex() {}
    PosNormalTex(const DirectX::SimpleMath::Vector3& pos, const DirectX::SimpleMath::Vector3& normal, const DirectX::SimpleMath::Vector2& tex) : pos(pos), normal(normal), tex(tex) {}
    DirectX::SimpleMath::Vector3 pos;
    DirectX::SimpleMath::Vector3 normal;
    DirectX::SimpleMath::Vector2 tex;
  };
}
