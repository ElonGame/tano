#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 upper;
  float4 lower;
};

// entry-point: ps
float4 PsBackground(VSQuadOut input) : SV_TARGET
{
  float t = input.uv.y;
  t = smoothstep(0.1, 0.9, t);
  return lerp(upper, lower, smoothstep(0.1, 0.9, t));
}
