#include "common.hlsl"

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
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

//------------------------------------------------------
// mesh
//------------------------------------------------------

// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  return res;
}

// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  float3 d = normalize(float3(0.5, 0.7, -1));
  float3 amb = float3(0.05, 0.05, 0.05);
  float dff = saturate(dot(d, p.normal));
  return float4(amb + dff * float3(0.1, 0.5, 0.2), 1);
}
