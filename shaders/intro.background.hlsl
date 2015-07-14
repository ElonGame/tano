#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 inner;
  float4 outer;
};


float4 PsBackground(VSQuadOut input) : SV_TARGET
{
  float t = input.uv.x;
  if (t > 0.5f)
    return lerp(outer, inner, 2 * (t-0.5f));

  return lerp(inner, outer, 2 * t);
}
