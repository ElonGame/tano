#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
};

static float3 FOG_COLOR = float3(16, 24, 37) / 255;
static float3 SUN_COLOR = float3(128, 100, 120) / 255;
static float3 SUN_DIR = normalize(float3(0, -0.2, -1));
static float3 SUN_POS = float3(0, 0, 2000);
static float SUN_POWER = 30;

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, SUN_POWER));
  return fogColor;
}

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
