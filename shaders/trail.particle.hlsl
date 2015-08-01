#include "common.hlsl"

cbuffer G : register(b0)
{
  float4x4 world;
  float4x4 viewProj;
  float3 cameraPos;
  float2 numParticles;
};

struct VsParticleIn
{
  float3 pos : Position;
};

struct GsParticleIn
{
  float3 pos : Position;
  float id : TexCoord0;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
  float fade : TexCoord1;
};

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

// entry-point: vs
GsParticleIn VsParticle(VsParticleIn v, uint vertexID : SV_VertexID)
{
  GsParticleIn res;
  res.pos = v.pos;
  res.id = vertexID;
  return res;
}

[maxvertexcount(4)]
// entry-point: gs
void GsParticle(point GsParticleIn input[1], uint id : SV_PrimitiveID, inout TriangleStream<VsParticleOut> outStream)
{
  // Note, the DirectX strip order differs from my usual order. It might be
  // a good idea to change my stuff..
  // 1--3
  // |  |
  // 0--2

  float4x4 worldViewProj = mul(world, viewProj);

  float3 pos = input[0].pos;
  float3 dir = normalize(cameraPos - pos);
  float3 right = cross(dir, float3(0,1,0));
  float3 up = cross(right, dir);

  VsParticleOut p;
  p.fade = input[0].id / numParticles.x;
  float s = 0.75;
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

static float SoftParticleContrast = 2.0;
static float intensity = 1.0;
static float zEpsilon = 0.0;

// entry-point: ps
float4 PsParticle(VsParticleOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  col = 0.1 * float4(0.2, 0.2, 0.3, 1) * col;
  return float4(p.fade * col.rgb, col.a);
}
