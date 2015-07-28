#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix viewProj;
  matrix objWorld;
};

struct VsIn
{
  float3 pos : Position;
};

struct VsOut
{
  float4 pos : SV_Position;
};

VsOut VsGreets(VsIn v)
{
  VsOut res;
  matrix worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  return res;
}

float4 PsGreets(VsOut v) : SV_Target
{
  return 1;
}
