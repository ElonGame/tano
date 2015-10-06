#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 tonemap;
  float4 time; // x = local-time, y = text fade
};

float4 DistortOld1(Texture2D tex, float2 uv, float time)
{
  float angle = 0;
  int NUM_SAMPLES = 10;
  float angleInc = 3.1415926 / NUM_SAMPLES * uv.x * uv.y;
  float4 res = float4(0,0,0,0);
  float radius = 0.01 * time;
  for (int i = 0; i < NUM_SAMPLES; ++i)
  {
    float2 ofs = sin(radius) * float2(0.5 + 0.5 * cos(angle), 0.5 + 0.5 * sin(angle));
    res += Texture4.Sample(LinearWrap, uv + ofs);
    angle += angleInc;
  }
  return res;
}

float4 Distort(Texture2D tex, float2 uv, float time)
{
  float2 center = float2(0.8, 0.8);
  float dist = saturate(1 - 0.5 * time) * length(center - uv);
  float2 delta = center - uv;

  float2 ofs = dist * float2(0.5 + 0.5 * cos(delta.x), 0.5 + 0.5 * sin(delta.y));
  return Texture4.Sample(LinearWrap, uv + ofs);
}


// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // textures used:
  // 0 - background
  // 1 - lines, normal
  // 2 - lines, blurred
  // 4 - text
  // 5 - textBlurred
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 backgroundCol = Texture0.Sample(LinearSampler, uv);
  float4 bgBlur = Texture3.Sample(LinearSampler, uv);

  float4 textCol = Texture4.Sample(LinearSampler, uv);
  float4 textBlurCol = Texture5.Sample(LinearSampler, uv);

  return Distort(Texture4, uv, time.x);

  // return textCol;

  float lum = Luminance(bgBlur.rgb);
  backgroundCol = backgroundCol + pow(abs(bgBlur), 1.5);

  // float4 linesR = Texture1.Sample(LinearSampler, uv);
  // linesR.xyzw = linesR.xxxw;
  // float4 linesBlurR = Texture2.Sample(LinearSampler, uv);
  // float lumBlur = pow(abs(Luminance(linesBlurR.rgb)), 1.5);

  float textBlurLum = pow(abs(Luminance(textBlurCol.rgb)), 1.5);
  float textLum = Luminance(textCol.rgb);
  float4 col = 0.1 * backgroundCol;
  float4 finalTextCol = float4(0,0,0,0);
  if (textLum > 0)
  {
    finalTextCol = lerp(col, 2 * textCol, 0.8);
  }

  float r = 0.5 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  col = smoothstep(0, 10, time.x) * (r * col + finalTextCol);

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);
  return col;
}


