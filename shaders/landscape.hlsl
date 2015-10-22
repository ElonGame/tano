#include "common.hlsl"

struct PsColBrightnessOut
{
  float4 col : SV_Target0;
  // rgb = emissive
  // a = bloom factor
  float4 emissive : SV_Target1;
};

static float4 BOID_COLOR = float4(0.4, 0.2, 0.2, 1);
static float3 FOG_COLOR = float3(16, 24, 37) / 4096;
static float FOG_SCALE = 0.01;
static float3 SUN_COLOR = float3(140, 140, 100) / 256;
static float3 SUN_DIR = normalize(float3(-1, -0.1, 1));
static float3 SUN_POS = float3(0, 0, 2000);
static float SUN_POWER = 100;

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, SUN_POWER));
  return fogColor;
}
