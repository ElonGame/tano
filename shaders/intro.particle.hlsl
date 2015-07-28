#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix world;
  matrix viewProj;
};

struct VsParticleIn
{
  float4 pos : Position;
};

struct GsParticleIn
{
  float4 pos : Position;
  float4 data : TexCoord0;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float4 uv : TexCoord0;
};


// 1--2
// |  |
// 0--3

static float2 uvsVtx[4] =
{
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

// entry-point: vs
GsParticleIn VsParticle(VsParticleIn v)
{
  GsParticleIn res;
  res.pos = v.pos; 
  res.data.x = v.pos.w;
  res.pos.w = 1;
  return res;
}

[maxvertexcount(4)]
// entry-point: gs
void GsParticle(point GsParticleIn input[1], inout TriangleStream<VsParticleOut> outStream)
{
  // Note, the DirectX strip order differs from my usual order. It might be
  // a good idea to change my stuff..
  // 1--3
  // |  |
  // 0--2

  matrix worldViewProj = mul(world, viewProj);

  float3 pos = input[0].pos.xyz;
  float4 data = input[0].data;
  float3 dir = float3(0,0,-1);
  float3 right = float3(1,0,0);
  float3 up = float3(0,1,0);

  VsParticleOut p;
  float s = 10;
  float3 p0 = float3(pos - s * right - s * up);
  float3 p1 = float3(pos - s * right + s * up);
  float3 p2 = float3(pos + s * right - s * up);
  float3 p3 = float3(pos + s * right + s * up);

  p.pos = mul(float4(p0, 1), worldViewProj);
  p.uv.xy = uvsVtx[0];
  p.uv.z = data.x;
  p.uv.w = 0;
  outStream.Append(p);

  p.pos = mul(float4(p1, 1), worldViewProj);
  p.uv.xy = uvsVtx[1];
  outStream.Append(p);

  p.pos = mul(float4(p2, 1), worldViewProj);
  p.uv.xy = uvsVtx[2];
  outStream.Append(p);

  p.pos = mul(float4(p3, 1), worldViewProj);
  p.uv.xy = uvsVtx[3];
  outStream.Append(p);
}

// entry-point: ps
float4 PsParticle(VsParticleOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  return (1 - p.uv.z) * float4(col.rgb, col.g);
}