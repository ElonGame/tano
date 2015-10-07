#include "common.hlsl"
#include "noise_lib.hlsl"

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

float2 ToPolar(float2 v)
{
  return float2(length(v), atan2(v.y, v.x));
}

float2 ToCartesian(float2 p)
{
  // x = radius, y = angle
  return float2(p.x * cos(p.y), p.x * sin(p.y));
}

float4 Distort(Texture2D tex, float2 uv, float time)
{
  float2 center = float2(0.8, 0.9);
  float2 xx = uv - center;
  float r = 0;
  float f = 3;
  for (int i = 0; i < 4; ++i)
  {
    float n = 1 / f * noise(xx * f + float2(5, 0));
    r += n;
    f *= 2;
  }

  float2 polar = ToPolar(xx);
  float timeScale = pow(saturate(1 - time / 5), 3);
  polar.x += 1 * timeScale * r;
  polar.y += 10 * timeScale * r;
  float2 uv2r = ToCartesian(float2(polar.x, polar.y)) + center;
  float2 uv2g = ToCartesian(float2(polar.x, polar.y + 0.005 * timeScale)) + center;
  float2 uv2b = ToCartesian(float2(polar.x, polar.y - 0.005 * timeScale)) + center;

  float colr = (1 - timeScale) * Texture4.Sample(LinearSampler, uv2r).r;
  float colg = (1 - timeScale) * Texture4.Sample(LinearSampler, uv2g).g;
  float colb = (1 - timeScale) * Texture4.Sample(LinearSampler, uv2b).b;
  return float4(colr, colg, colb, 1);
  // float4 col = (1 - timeScale) * Texture4.Sample(LinearSampler, uv2);
  // return col;
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

  float4 textCol = float4(0,0,0,0);
  // float4 textBlurCol = Texture5.Sample(LinearSampler, uv);

  if (time.x > 9.5)
  {
    textCol = Distort(Texture4, uv, time.x - 9.5);
  }

  // return textCol;

  float lum = Luminance(bgBlur.rgb);
  backgroundCol = backgroundCol + pow(abs(bgBlur), 1.5);

  // float4 linesR = Texture1.Sample(LinearSampler, uv);
  // linesR.xyzw = linesR.xxxw;
  // float4 linesBlurR = Texture2.Sample(LinearSampler, uv);
  // float lumBlur = pow(abs(Luminance(linesBlurR.rgb)), 1.5);

  // float textBlurLum = pow(abs(Luminance(textBlurCol.rgb)), 1.5);
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


