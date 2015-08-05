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
  float3 pos : Position;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
};

//------------------------------------------------------------------------------
// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;

  float4 pos = mul(float4(v.pos, 1), objWorld);
  res.pos = mul(pos, viewProj);   
  return res; 
}

// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  return 1;
  // return saturate(dot(p.normal_ws.xyz, normalize(float3(0, 1, -1))));
}
