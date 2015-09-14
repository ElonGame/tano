#include "common.hlsl"

struct PsColBrightnessOut
{
  float4 col : SV_Target0;
  // rgb = emissive
  // a = bloom factor
  float4 emissive : SV_Target1;
};

static float4 BOID_COLOR = float4(0.4, 0.2, 0.2, 1);
// static float3 FOG_COLOR = 0.5 * float3(0.5, 0.6, 0.7);
static float3 FOG_COLOR = float3(16, 24, 37) / 255;
// static float3 SUN_COLOR = 0.5 * float3(1.5, 0.9, 0.3);
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
