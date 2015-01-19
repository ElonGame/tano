Texture2D Texture0 : register(t0);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix viewProj;
  float4 tint;
};

struct VsIn
{
  float3 pos : Position;
  float2 tex : TexCoord;
};

struct VsOut
{
  float4 pos : SV_Position;
  float2 tex : TexCoord;
};

VsOut VsMain(VsIn v)
{
  VsOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.tex = v.tex;
  return res;
}

float4 PsMain(VsOut p) : SV_Target
{
//  return p.tex.y;
  float4 col = Texture0.Sample(PointSampler, p.tex);
//  return tint.r;
  return tint * col;
//  return float4(col, 1);

  float s = sin(p.pos.x/100);
  float c = cos(p.pos.x/100);
  return 1;
//  return float4(pow(c,abs(s)),c, s,0);
}
