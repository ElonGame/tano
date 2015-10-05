#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 tonemap;
  float4 time; // x = local-time, y = text fade
};


// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // textures used:
  // 0 - background
  // 1 - lines, normal
  // 2 - lines, blurred
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float4 bgBlur = Texture3.Sample(PointSampler, uv);

  float lum = Luminance(bgBlur.rgb);
  backgroundCol = backgroundCol + pow(abs(bgBlur), 1.5);

  float4 linesR = Texture1.Sample(PointSampler, uv);
  linesR.xyzw = linesR.xxxw;
  //linesR -= bgBlur;
  float4 linesBlurR = Texture2.Sample(PointSampler, uv);

  float lumBlur = pow(abs(Luminance(linesBlurR.rgb)), 1.5);
  float4 col = 0.1 * backgroundCol + linesR * 0.1 + float4(0.3, 0.3, 0.2, 1) * lumBlur;

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);

  // vignette
  float r = 0.5 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return smoothstep(0, 10, time.x) * r * col;
}


