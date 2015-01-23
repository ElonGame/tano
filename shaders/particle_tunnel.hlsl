Texture2D Texture0 : register(t0);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix viewProj;
  float4 tint;
  float4 inner;
  float4 outer;
};

//------------------------------------------------------
struct VsParticleIn
{
  float3 pos : Position;
  float3 uv : TexCoord;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float3 uv : TexCoord;
};

struct VsTextIn
{
  float3 pos : Position;
};

struct VsTextOut
{
  float4 pos : SV_Position;
};

struct VSQuadOut
{
    float4 pos : SV_Position;
    float2 uv: TexCoord;
};

//------------------------------------------------------
// particles
//------------------------------------------------------

VsParticleOut VsParticle(VsParticleIn v)
{
  VsParticleOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.uv = v.uv;
  return res;
}

float4 PsParticle(VsParticleOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  return (1 - p.uv.z) * float4(col.rgb, col.g);
}

//------------------------------------------------------
// background
//------------------------------------------------------

// outputs a full screen triangle with screen-space coordinates
// input: three empty vertices
VSQuadOut VsQuad(uint vertexID : SV_VertexID)
{
    VSQuadOut result;
  // ID=0 -> Pos=[-1,-1], Tex=[0,0]
  // ID=1 -> Pos=[ 3,-1], Tex=[2,0]
  // ID=2 -> Pos=[-1,-3], Tex=[0,2]
    result.uv = float2((vertexID << 1) & 2, vertexID & 2);
    result.pos = float4(result.uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
    return result;
}

float4 PsBackground(VSQuadOut input) : SV_TARGET
{
  float t = input.uv.x;
  if (t > 0.5f)
    return lerp(outer, inner, 2 * (t-0.5f));

  return lerp(inner, outer, 2 * t);
}

//------------------------------------------------------
// text
//------------------------------------------------------

VsTextOut VsText(VsTextIn v)
{
  VsTextOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  return res;
}

float4 PsText(VsTextOut p) : SV_Target
{
  return 1;
}

//------------------------------------------------------
// composite
//------------------------------------------------------
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  float2 xx = -1 + 2 * uv;
  float r = 0.5 + 0.9 - sqrt(xx.x*xx.x + xx.y*xx.y);
  return r * col;
  return col;
}
