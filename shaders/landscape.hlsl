#include "common.hlsl"

Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 dim;
};

struct VsLandscapeIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsLandscapeOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
};

//------------------------------------------------------
// sky
//------------------------------------------------------
float4 PsSky(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 r = -1 + 2 * uv;

  float s = pow(0.01, length(r));
  return float4(s, s, s, s);
}

//------------------------------------------------------
// landscape
//------------------------------------------------------

VsLandscapeOut VsLandscape(VsLandscapeIn v)
{
  VsLandscapeOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  return res;
}

float4 PsLandscape(VsLandscapeOut p) : SV_Target
{
  float3 d = normalize(float3(0.5, 0.7, -1));
  float3 amb = float3(0.05, 0.05, 0.05);
  float dff = saturate(dot(d, p.normal));
  return float4(amb + dff * float3(0.1, 0.1, 1), 1);
}

//------------------------------------------------------
// edge detection
//------------------------------------------------------
float4 PsEdgeDetect(VSQuadOut input) : SV_Target
{
  // Sobel filter
  float2 ofs = 1 / dim.xy;
  float3 l00 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, -ofs.y)).rgb;
  float3 l01 = Texture0.Sample(PointSampler, input.uv + float2(     0, -ofs.y)).rgb;
  float3 l02 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, -ofs.y)).rgb;

  float3 l10 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, 0)).rgb;
  float3 l11 = Texture0.Sample(PointSampler, input.uv + float2(     0, 0)).rgb;
  float3 l12 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, 0)).rgb;

  float3 l20 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, +ofs.y)).rgb;
  float3 l21 = Texture0.Sample(PointSampler, input.uv + float2(     0, +ofs.y)).rgb;
  float3 l22 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, +ofs.y)).rgb;
  
  float3 gx = +1 * l00 -1 * l02 +2 * l10 -2 * l12 +1 * l20 -1 * l22;
  float3 gy = +1 * l00 +2 * l01 +1 * l02 -1 * l20 -2 * l21 -1 * l22;
  
  float3 e = sqrt(gx*gx+gy*gy);
  float t = max(e.x, e.y);
  return t > 0.2 ? 1 : 0;
}

//------------------------------------------------------
// composite
//------------------------------------------------------
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float4 edge = Texture1.Sample(PointSampler, uv);
  return max(backgroundCol, edge);
}

