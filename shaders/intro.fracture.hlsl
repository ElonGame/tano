#include "common.hlsl"

cbuffer PerFrame : register(b0)
{
//  float4 tonemap;
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 tint;
  float4 inner;
  float4 outer;
  float4 dim;
  float4 viewDir;
  float4 camPos;
  float4 dofSettings; // near-z-start, near-z-end, far-z-start, far-z-end
  float4 time; // x = local-time, y = text fade
  float3 cameraPos;
};

cbuffer PerObject : register(b2)
{
  matrix objWorld;
};

struct VsFractureIn
{
  float3 pos : Position;
  float3 normal : Normal;
  float2 uv : TexCoord0;
};

struct VsFractureOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

VsFractureOut VsFracture(VsFractureIn v)
{
  VsFractureOut res;

  matrix worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.uv = v.uv;
  return res;
}

float4 PsFracture(VsFractureOut v) : SV_Target
{
  return Texture0.Sample(LinearSampler, v.uv.xy);
}
