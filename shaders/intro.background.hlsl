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


float4 PsBackground(VSQuadOut input) : SV_TARGET
{
  float t = input.uv.x;
  if (t > 0.5f)
    return lerp(outer, inner, 2 * (t-0.5f));

  return lerp(inner, outer, 2 * t);
}
