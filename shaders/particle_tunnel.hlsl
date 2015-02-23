#include "common.hlsl"

Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
Texture2D Texture3 : register(t3);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 tint;
  float4 inner;
  float4 outer;
  float4 dim;
  float4 viewDir;
  float4 camPos;
  float4 dofSettings; // near-z-start, near-z-end, far-z-start, far-z-end
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
  float4 tex : TexCoord0;    // xy = uv, z = dof
};

struct PsLinesOut
{
  float4 col  : SV_Target0;
  float dof   : SV_Target1;
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

float CalcDof(float z)
{
  float nearStart = 500;
  float nearEnd = 2000;

  float farStart = 4000;
  float farEnd = 5500;
  // blend between 0..1 ... 1..0 
  return min(smoothstep(nearStart, nearEnd, z), 1 - smoothstep(farStart, farEnd, z));
}

void OutputVtx(float3 v, float2 tex, inout TriangleStream<GsLinesOut> stream)
{
  GsLinesOut res;
  res.pos = mul(float4(v, 1), viewProj);
  res.tex.xy = tex.xy;
  float3 dist = v - camPos.xyz;
  res.tex.zw = CalcDof(length(dist));
  stream.Append(res);
}


[maxvertexcount(8)]
void GsLines(line VsLinesOut input[2], inout TriangleStream<GsLinesOut> stream)
{
  /*
      1--3
      |  |
      0--2
  */

  float3 a = input[0].pos;
  float3 b = input[1].pos;
  float3 mid = (a+b)/2;

  float3 right = normalize(b-a);
  float3 dir = normalize(camPos.xyz-mid);
  float3 up = normalize(cross(right, dir));

  float h = 15;
  float w = 20;
  float3 v0 = a - w * right - h * up;
  float3 v1 = a - w * right + h * up;
  float3 v2 = b + w * right - h * up;
  float3 v3 = b + w * right + h * up;

  OutputVtx(v0, float2(0.5, 1), stream);
  OutputVtx(v1, float2(0.5, 0), stream);
  OutputVtx(v2, float2(0.5, 1), stream);
  OutputVtx(v3, float2(0.5, 0), stream);
}


PsLinesOut PsLines(GsLinesOut input)
{
  float3 col = Texture0.Sample(PointSampler, input.tex.xy).rgb;
  float aa = length(col);
  float bb = pow(aa, 5);
  // the output is monochrome, so output the DOF in g
  PsLinesOut res;
  res.col = float4(bb, bb, bb, 0);
  res.dof = input.tex.z;
  return res;
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
  // textures used:
  // 0 - background
  // 1 - lines, depth
  // 2 - lines, blurred
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
//  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float4 lines = Texture0.Sample(PointSampler, uv);
  float4 linesDepth = Texture1.Sample(PointSampler, uv);
  float4 linesBlur = Texture2.Sample(PointSampler, uv);

/*  
  float4 sample = Texture1.Sample(PointSampler, uv);
  float4 orgCol = sample;
  float dof = saturate(1 - length(orgCol));
  dof = saturate(CalcDof(600));
  dof = 1 - pow(length(xx), 20);
  float4 col = lerp(blurCol, orgCol, dof);
*/
  //float4 col = backgroundCol;
  float dof = linesDepth.x;
  float4 col = lerp(linesBlur, lines, saturate(dof));
//  col = lines;

  //col = col + col2;
  // vignette
  float r = 0.5 + 0.9 - sqrt(xx.x*xx.x + xx.y*xx.y);
  return r * col;
}
