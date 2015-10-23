#include "common.hlsl"

struct PsColBrightnessOut
{
  float4 col : SV_Target0;
  // rgb = emissive
  // a = bloom factor
  float4 emissive : SV_Target1;
};

static float3 FOG_COLOR = float3(16, 24, 37) / 4096;
static float FOG_SCALE = 0.015;
static float3 SUN_COLOR = float3(140, 140, 160) / 200;
static float3 SUN_DIR = normalize(float3(-1, -0.1, 1));
static float3 SUN_POS = float3(0, 0, 2000);
static float SUN_POWER = 150;

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, SUN_POWER));
  return fogColor;
}
