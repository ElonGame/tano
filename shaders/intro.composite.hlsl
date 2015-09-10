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
  backgroundCol = backgroundCol + pow(bgBlur, 1.5);

  float4 linesR = Texture1.Sample(PointSampler, uv);
  float4 linesBlurR = Texture2.Sample(PointSampler, uv);

  float cc = 5 * time.z;

  float4 linesG = Texture1.Sample(PointSampler, uv + cc * 0.01);
  float4 linesBlurG = Texture2.Sample(PointSampler, uv + cc * 0.01);

  float4 linesB = Texture1.Sample(PointSampler, uv + cc * 0.02);
  float4 linesBlurB = Texture2.Sample(PointSampler, uv + cc * 0.02);

  float dofBlur = linesBlurR.g;
  linesR.xyzw = linesR.xxxw; linesBlurR.xyzw = linesBlurR.xxxw;
  linesG.xyzw = linesG.xxxw; linesBlurG.xyzw = linesBlurG.xxxw;
  linesB.xyzw = linesB.xxxw; linesBlurB.xyzw = linesBlurB.xxxw;

  float4 tmpR = lerp(linesBlurR, linesR, saturate(dofBlur));
  float4 tmpG = lerp(linesBlurG, linesG, saturate(dofBlur));
  float4 tmpB = lerp(linesBlurB, linesB, saturate(dofBlur));

  float4 tmp = (1 + time.w) * float4(tmpR.x, tmpG.y, tmpB.z, tmpR.w);
  float4 fadeTmp = (1 - smoothstep(0, 1, time.y)) * tmp;
  float lumBlur = pow(abs(Luminance(linesBlurR.rgb)), 1.5);
  float4 col = 0.1 * backgroundCol + linesR * 0.2 + float4(0.4, 0.4, 0.3, 1) * lumBlur;

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);

  // vignette
  float r = 0.5 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return smoothstep(0, 10, time.x) * r * col;
}


