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

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

// entry-point: vs
VsParticleIn VsParticle(VsParticleIn v)
{
  VsParticleIn res;
  res.pos = v.pos;
  return res;
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

// entry-point: ps
PsColBrightnessOut PsParticle(VsParticleOut p)
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  return (1 - p.uv.z) * float4(col.rgb, col.g);
}
