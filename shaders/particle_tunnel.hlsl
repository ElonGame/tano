#include "common.hlsl"

Texture2D Texture0 : register(t0);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix viewProj;
  float4 tint;
  float4 inner;
  float4 outer;
  float2 dim;
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
  float3 pos  : Position;
  uint idx    : SV_VertexId;
};

struct VsTextOut
{
  float4 pos : SV_Position;
  noperspective float3 dist : COLOR;
};

struct VsLinesIn
{
  float3 pos : Position;
};

struct VsLinesOut
{
  float3 pos : Position;
};

struct GsLinesOut
{
  float4 pos : SV_Position;
  float4 a : PointA;
  float4 b : PointB;
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

float4 PsBackground(VSQuadOut input) : SV_TARGET
{
  float t = input.uv.x;
  if (t > 0.5f)
    return lerp(outer, inner, 2 * (t-0.5f));

  return lerp(inner, outer, 2 * t);
}

//------------------------------------------------------
// lines
//------------------------------------------------------

VsLinesOut VsLines(VsLinesIn v)
{
  VsLinesOut res;
  res.pos = v.pos;
  return res;
}

void OutputVtx(float3 v, float3 a, float3 b, inout TriangleStream<GsLinesOut> stream)
{
  GsLinesOut res;
  res.pos = mul(float4(v, 1), viewProj);
  res.a = mul(float4(a, 1), viewProj);
  res.b = mul(float4(b, 1), viewProj);
  res.a /= res.a.w;
  res.b /= res.b.w;
  stream.Append(res);
}

[maxvertexcount(4)]
void GsLines(line VsLinesOut input[2], inout TriangleStream<GsLinesOut> stream)
{
  // output a triangle strip for the current line
  float3 a = input[0].pos;
  float3 b = input[1].pos;
  float3 dir = normalize(b-a);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);

  float h = 5;

  OutputVtx(a - h * up, a, b, stream);
  OutputVtx(a + h * up, a, b, stream);
  OutputVtx(b - h * up, a, b, stream);
  OutputVtx(b + h * up, a, b, stream);
}

// Return distance from point 'p' to line segment 'a b':
float line_distance(float2 p, float2 a, float2 b)
{
    float dist = distance(a,b);
    float2 v = normalize(b-a);
    float t = dot(v,p-a);
    float2 spinePoint;
    if (t > dist) spinePoint = b;
    else if (t > 0.0) spinePoint = a + t*v;
    else spinePoint = a;
    return distance(p,spinePoint);
}

float4 PsLines(GsLinesOut input) : Sv_Target
{
  float2 a = input.a.xy;
  float2 b = input.b.xy;
  // convert from [-1..1][-1..1] to [0..w][h..0]
  a.x = (1 + a.x) / 2 * dim.x; 
  a.y = (1 - (1 + a.y) / 2) * dim.y;
  b.x = (1 + b.x) / 2 * dim.x; 
  b.y = (1 - (1 + b.y) / 2) * dim.y;

  float d = line_distance(input.pos.xy, a, b);
  float t = 1.0 - 12 / d;
  return float4(t, t, t, 1);
  return t;
  return float4(t, t, t, 1);
}

//------------------------------------------------------
// text
//------------------------------------------------------

VsTextOut VsText(VsTextIn v)
{
  VsTextOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);

  switch (v.idx % 3)
  {
    case 0: res.dist = float3(0,0,1); break;
    case 1: res.dist = float3(0,1,0); break;
    case 2: res.dist = float3(1,0,0); break;
  }

  return res;
}

float4 PsText(VsTextOut p) : SV_Target
{
  float d = min(p.dist.x, min(p.dist.y, p.dist.z));
  float f = 0.3;
  return d < 0.02 ? float4(f, f, f, f) : float4(0.1,0.1,0.2,0.1);
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
