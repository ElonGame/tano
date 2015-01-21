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
  float4 col = Texture0.Sample(PointSampler, p.tex);
  return float4(col.rgb, col.g);
}

//------------------------------------------------------
struct VSQuadOut
{
    float4 pos : SV_Position;
    float2 uv: TexCoord;
};

// outputs a full screen triangle with screen-space coordinates
// input: three empty vertices
VSQuadOut VsBackground(uint vertexID : SV_VertexID)
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
