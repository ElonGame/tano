#include "common.hlsl"

cbuffer G : register(b0)
{
  matrix world;
  matrix viewProj;
  float3 cameraPos;
};

struct VsParticleIn
{
  float3 pos : Position;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

// 1--2
// |  |
// 0--3

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

// entry-point: vs
VsParticleIn VsParticle(VsParticleIn v)
{
  return v;
}

[maxvertexcount(4)]
// entry-point: gs
void GsParticle(point VsParticleIn input[1], inout TriangleStream<VsParticleOut> outStream)
{
  // Note, the DirectX strip order differs from my usual order. It might be
  // a good idea to change my stuff..
  // 1--3
  // |  |
  // 0--2

  matrix worldViewProj = mul(world, viewProj);

  float3 pos = input[0].pos.xyz;
  float3 dir = normalize(cameraPos - pos);
  float3 right = cross(dir, float3(0,1,0));
  float3 up = cross(right, dir);

  VsParticleOut p;
  float s = 1.75;
  float3 p0 = float3(pos - s * right - s * up);
  float3 p1 = float3(pos - s * right + s * up);
  float3 p2 = float3(pos + s * right - s * up);
  float3 p3 = float3(pos + s * right + s * up);
  p.pos = mul(float4(p0, 1), worldViewProj);
  p.uv = uvsVtx[0];
  outStream.Append(p);

  p.pos = mul(float4(p1, 1), worldViewProj);
  p.uv = uvsVtx[1];
  outStream.Append(p);

  p.pos = mul(float4(p2, 1), worldViewProj);
  p.uv = uvsVtx[2];
  outStream.Append(p);

  p.pos = mul(float4(p3, 1), worldViewProj);
  p.uv = uvsVtx[3];
  outStream.Append(p);
}

// entry-point: ps
float4 PsParticle(VsParticleOut p) : SV_Target
{
  // Texture1 = zbuffer
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  return ToFloat4(col, col.r);
}
