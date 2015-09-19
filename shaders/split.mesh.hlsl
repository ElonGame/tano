#include "common.hlsl"
#include "split.common.hlsl"

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
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
};

//------------------------------------------------------------------------------
// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;

  float4 pos = mul(float4(v.pos, 1), objWorld);
  res.pos = mul(pos, viewProj);
  res.normal = v.normal;
  return res; 
}

// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  float3 n = normalize(p.normal);
  return saturate(0.1 + dot(n, -SUN_DIR));
}
