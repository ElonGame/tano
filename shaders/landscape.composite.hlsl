#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 tonemap; // x = exposure/lumAvg, y = min_white
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

  // float4 col = bgCol + 1 * emm.a * bgColBlurred + 0.3 * lensFlare;
  float4 col = bgCol + 1.5 * pow(bgColBlurred, 1 + 2 * emm.a) + 0.5 * lensFlare;

  float exposure = tonemap.x;
  float minWhite = tonemap.y;

  col = ToneMapReinhard(col, exposure, minWhite);
  
   // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * col;
}
