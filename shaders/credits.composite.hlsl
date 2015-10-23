#include "common.hlsl"

cbuffer P : register(b0)
{
  float4 tonemap; // z = exposure/lumAvg, w = min_white
  float4 time; // x = local-time
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 col = Texture0.Sample(LinearSampler, uv);
  float4 blur = Texture1.Sample(LinearSampler, uv);
  float4 credits = Texture2.Sample(LinearSampler, uv);

  float exposure = tonemap.x;
  float minWhite = tonemap.y;

  float creditsLum = Luminance(credits.xyz);

  col = col + smoothstep(0.0, 1, 3 * blur);
  if (creditsLum > 0.1)
  {
    col += smoothstep(146, 152, time.y) * credits;
  }

  col = ToneMapReinhard(col, exposure, minWhite);

  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float fadeIn = smoothstep(144, 145, time.y);
  float fadeOut = 1 - smoothstep(155, 160, time.y);
  return min(fadeIn, fadeOut) * col;
}
