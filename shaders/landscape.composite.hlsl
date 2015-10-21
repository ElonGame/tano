#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 tonemap; // x = exposure/lumAvg, y = min_white
  float4 time; // x = local-time
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // Texture0 : color
  // Texture1 : color blurred
  // Tetxure2 : emissive blurred
  // Texture3 : lensflare
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

  float4 bgCol = Texture0.Sample(PointSampler, uv);
  float4 bgColBlurred = Texture1.Sample(PointSampler, uv);
  float4 emm = Texture2.Sample(LinearSampler, uv);
  float4 lensFlare = Texture3.Sample(LinearSampler, uv);

  float4 col = bgCol + 1.5 * pow(abs(bgColBlurred), 1 + 2 * emm.a) + 0.5 * lensFlare;

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  float fade = tonemap.z;

  //col = fade * col;
  col = ToneMapReinhard(col, fade * exposure, minWhite);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  col = r * col;

   // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float tt = smoothstep(0, 0.5, time.x);
  return lerp(float4(0,0,0,0), col, tt);
}
