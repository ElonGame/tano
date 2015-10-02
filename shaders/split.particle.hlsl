#include "common.hlsl"

cbuffer G : register(b0)
{
  matrix world;
  matrix view;
  matrix viewProj;
  float3 cameraPos;
};

struct VsParticleIn
{
  float3 pos : Position;
  float fade : Texture0;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
  float depthVS : DepthVS;
  float fade : Texture0;
};

struct PSOut
{
  float4 col : SV_Target0;
  float4 revealage : SV_Target1;
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
  p.fade = input[0].fade;
  float s = 0.75;
  float3 p0 = float3(pos - s * right - s * up);
  float3 p1 = float3(pos - s * right + s * up);
  float3 p2 = float3(pos + s * right - s * up);
  float3 p3 = float3(pos + s * right + s * up);
  p.pos = mul(float4(p0, 1), worldViewProj);
  p.uv = uvsVtx[0];
  p.depthVS = mul(float4(p0, 1), view).z;
  outStream.Append(p);

  p.pos = mul(float4(p1, 1), worldViewProj);
  p.uv = uvsVtx[1];
  p.depthVS = mul(float4(p1, 1), view).z;
  outStream.Append(p);

  p.pos = mul(float4(p2, 1), worldViewProj);
  p.uv = uvsVtx[2];
  p.depthVS = mul(float4(p2, 1), view).z;
  outStream.Append(p);

  p.pos = mul(float4(p3, 1), worldViewProj);
  p.uv = uvsVtx[3];
  p.depthVS = mul(float4(p3, 1), view).z;
  outStream.Append(p);
}

// entry-point: ps
PSOut PsParticle(VsParticleOut p)
{
  float fade = p.fade;
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(LinearSampler, uv);
  float4 orgCol = col;

  PSOut res;
  float a = 0.5;
  col.rgb *= a;
  float z = p.depthVS;
  float w = pow(a, 1.0) * clamp(0.3 / (1e-5 + pow(z / 200, 4.0)), 1e-2, 3e3);

  res.col = fade * w * float4(col.rgb, 0.1);
  res.revealage = fade * a;
  return res;
}
