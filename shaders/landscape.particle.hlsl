#include "landscape.hlsl"

cbuffer P : register(b0)
{
  float4 nearFar : NEAR_FAR;
};

cbuffer G : register(b0)
{
  matrix world;
  matrix viewProj;
  float3 cameraPos;
};

struct VsParticleIn
{
  float4 pos : Position;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
  float z : TexCoord1;
  float alpha : TexCoord2;
};

// 1--2
// |  |
// 0--3

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

VsParticleIn VsParticle(VsParticleIn v)
{
  VsParticleIn res;
  res.pos = v.pos;
  return res;
}

[maxvertexcount(4)]
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
  p.alpha = 1;
  float s = 0.75;
  float3 p0 = float3(pos - s * right - s * up);
  float3 p1 = float3(pos - s * right + s * up);
  float3 p2 = float3(pos + s * right - s * up);
  float3 p3 = float3(pos + s * right + s * up);
  p.pos = mul(float4(p0, 1), worldViewProj);
  p.uv = uvsVtx[0];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p1, 1), worldViewProj);
  p.uv = uvsVtx[1];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p2, 1), worldViewProj);
  p.uv = uvsVtx[2];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p3, 1), worldViewProj);
  p.uv = uvsVtx[3];
  p.z = p.pos.z;
  outStream.Append(p);
}

static float SoftParticleContrast = 2.0;
static float intensity = 1.0;
static float zEpsilon = 0.0;

PsColBrightnessOut PsParticle(VsParticleOut p)
{
  // Texture0 = particle texture
  // Texture1 = zbuffer
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  col.g *= 0.6;
  col.b *= 0.1;
  float zBuf = Texture1.Load(int3(p.pos.x, p.pos.y, 0)).r;

  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float farClip = nearFar.y;
  float f_mul_n = nearFar.z;
  float f_sub_n = nearFar.w;
  float z = f_mul_n / ( farClip - zBuf * f_sub_n);
  float zdiff = (z - p.z);
  float c = smoothstep(0, 1, zdiff / SoftParticleContrast);
  if( c * zdiff <= zEpsilon )
      discard;

  PsColBrightnessOut res;
  res.col = intensity * c * col;
  res.extra.rgb = intensity * c * col.rgb;
  res.extra.a = 0;
  return res;
}
