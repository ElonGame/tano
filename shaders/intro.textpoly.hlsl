#include "common.hlsl"

cbuffer V : register(b0)
{
  matrix viewProj;
  matrix objWorld;
};

cbuffer P : register(b0)
{
  float4 params;
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
  float fade = params.x;
  fade += smoothstep(0.5, 1, params.y);
  float3 n = normalize(p.normal);
  float3 lightDir = float3(0, 0, -1);
  return fade * saturate(dot(normalize(p.normal), float3(0,0,-1)));
}

