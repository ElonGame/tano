#include "common.hlsl"
#include "split.common.hlsl"

cbuffer F : register(b0)
{
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
};

// entry-point: ps
float4 PsSky(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 r = -1 + 2 * uv;

  float f = 1;

  // calc camera base
  float3 dir = normalize(cameraLookAt - cameraPos);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);
  up = cross(right, dir);

  // calc ray dir 
  // this is done by determining which pixel on the image
  // plane the current pixel represents
  float aspectRatio = dim.x / dim.y;
  float3 rayDir = 2 * (-0.5 + p.pos.x / dim.x) * right + (2 * (-0.5 + p.pos.y / dim.y)) * up + f * dir;
  rayDir.y /= aspectRatio;
  rayDir = normalize(rayDir);

  float3 tmp = FogColor(rayDir);
  return float4(tmp, 1);
}
