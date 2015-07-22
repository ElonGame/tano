#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix viewProj;
};

cbuffer O : register(b1)
{
  matrix objWorld;
};

struct VsMeshIn
{
  float3 pos : SV_Position;
  float3 normal : Normal;
  float2 uv : TexCoord0;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;

  matrix worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.uv = v.uv;
  return res;
}

float4 PsMesh(VsMeshOut v) : SV_Target
{
  return 1;
}
