#include "common.hlsl"

struct PsColBrightnessOut
{
  float4 col : SV_Target0;
  // rgb = emissive
  // a = bloom factor
  float4 emissive : SV_Target1;
};

static float3 FOG_COLOR = float3(16, 24, 37) / 2000;
static float FOG_SCALE = 0.005;
static float3 SUN_COLOR = float3(180, 170, 120) / 200;
static float3 SUN_POS = float3(0, 0, 2000);
static float SUN_POWER = 40;

float3 FogColor(float3 rayDir, float3 sunDir)
{
  float sunAmount = max(0, dot(rayDir, -sunDir));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, SUN_POWER));
  return fogColor;
}
