#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix viewProj;
  matrix objWorld;
  float3 camPos;
};

struct VsIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsOut
{
  float4 pos : SV_Position;
  float4 normal : Normal;
};

// entry-point: vs
VsOut VsGreets(VsIn v)
{
  VsOut res;
  matrix worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), objWorld);
  return res;
}

// entry-point: ps
float4 PsGreets(VsOut v) : SV_Target
{
  return 0.1 + saturate(dot(-normalize(v.normal.xyz), normalize(float3(0, -1, 1))));
}
