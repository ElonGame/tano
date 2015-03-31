#include "common.hlsl"

Texture2D Texture0 : register(t0);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
  float3 cameraUp;
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

VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  return res;
}

float4 PsMesh(VsMeshOut p) : SV_Target
{
  //return float4(p.normal, 1);
  float3 d = normalize(float3(0.5, 0.7, -1));
  float3 amb = float3(0.05, 0.05, 0.05);
  float dff = saturate(dot(d, p.normal));
  return float4(amb + dff * float3(0.1, 0.5, 0.2), 1);
}
