#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix viewProj;
};

cbuffer O : register(b1)
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
  res.pos = mul(float4(v.pos, 1), objWorld);
  res.pos.xy /= 100;
  res.pos.zw = float2(0, 1);
  res.uv = v.uv;
  return res;
}

float4 PsFracture(VsFractureOut v) : SV_Target
{
  return Texture0.Sample(LinearSampler, v.uv.xy);
}
