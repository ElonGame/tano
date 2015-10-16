#include "common.hlsl"
#include "noise_lib.hlsl"

//------------------------------------------------------
struct VSTextIn
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

struct VSTextOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

cbuffer P : register(b0)
{
  float time;
  float brightness;
};

//------------------------------------------------------
// entry-point: vs
VSTextOut VsIntroText(VSTextIn v)
{
  return v;
}


//------------------------------------------------------
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

  float colr = (1 - timeScale) * tex.Sample(LinearSampler, uv2r).r;
  float colg = (1 - timeScale) * tex.Sample(LinearSampler, uv2g).g;
  float colb = (1 - timeScale) * tex.Sample(LinearSampler, uv2b).b;
  return float4(colr, colg, colb, 1);
}

//------------------------------------------------------
// entry-point: ps
float4 PsIntroTextDistort(VSQuadOut p) : SV_Target
{
  return brightness * Distort(Texture0, p.uv, time - 9);
}


