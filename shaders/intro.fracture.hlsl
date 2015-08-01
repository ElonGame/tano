#include "common.hlsl"

cbuffer F : register(b0)
{
  float4x4 viewProj;
};

cbuffer O : register(b1)
{
  float4x4 objWorld;
};

struct VsFractureIn
{
  float3 pos : SV_Position;
  float3 normal : Normal;
  float2 uv : TexCoord0;
};

struct VsFractureOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

// entry-point: vs
VsFractureOut VsFracture(VsFractureIn v)
{
  VsFractureOut res;

  float4x4 worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
//  res.pos = mul(float4(v.pos, 1), objWorld);
//  res.pos.xy /= 50;
//  res.pos.zw = float2(0, 1);
/*
  float3 tmp = v.pos;
  tmp.z = 0.5;
  res.pos = mul(float4(tmp, 1), objWorld);
  //res.pos = float4(v.pos, 1);
  res.pos.xyz /= 50;
*/
  res.uv = v.uv;
  return res;
}

// entry-point: ps
float4 PsFracture(VsFractureOut v) : SV_Target
{
  return Texture0.Sample(LinearSampler, v.uv.xy);
}
