static float3 FOG_COLOR = float3(16, 24, 37) / 255;
static float3 SUN_COLOR = float3(128, 100, 120) / 255;
static float3 SUN_DIR = normalize(float3(0, -0.2, -1));
static float SUN_POWER = 30;

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, SUN_POWER));
  return fogColor;
}
