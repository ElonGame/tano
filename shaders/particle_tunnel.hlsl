#include "common.hlsl"

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
  float4 time; // x = local-time, y = text fade
  float3 cameraPos;
};

//------------------------------------------------------
struct PsParticlesOut
{
  float4 col  : SV_Target0;
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
};


//------------------------------------------------------
// particles
//------------------------------------------------------
struct VsParticleIn
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

VsParticleIn VsParticle(VsParticleIn v)
{
  return v;
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


PsParticlesOut PsParticle(VsParticleOut p)
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  PsParticlesOut res;
  res.col = (1 - p.uv.z) * float4(col.rgb, col.g);
  return res;
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
  float nearStart = 1000;
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

  float h = 10;
  float w = 10;
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
  float bb = pow(aa, 4);
  PsLinesOut res;
  float dof = input.tex.z;
  res.col = float4(bb, dof, bb, bb);
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
  // 1 - lines, normal
  // 2 - lines, blurred
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 backgroundCol = Texture0.Sample(PointSampler, uv);

  float4 linesR = Texture1.Sample(PointSampler, uv);
  float4 linesBlurR = Texture2.Sample(PointSampler, uv);

  float cc = 5 * time.z;

  float4 linesG = Texture1.Sample(PointSampler, uv + cc * 0.01);
  float4 linesBlurG = Texture2.Sample(PointSampler, uv + cc * 0.01);

  float4 linesB = Texture1.Sample(PointSampler, uv + cc * 0.02);
  float4 linesBlurB = Texture2.Sample(PointSampler, uv + cc * 0.02);

  float dofBlur = linesBlurR.g;
  linesR.xyzw = linesR.xxxw; linesBlurR.xyzw = linesBlurR.xxxw;
  linesG.xyzw = linesG.xxxw; linesBlurG.xyzw = linesBlurG.xxxw;
  linesB.xyzw = linesB.xxxw; linesBlurB.xyzw = linesBlurB.xxxw;

  float4 tmpR = lerp(linesBlurR, linesR, saturate(dofBlur));
  float4 tmpG = lerp(linesBlurG, linesG, saturate(dofBlur));
  float4 tmpB = lerp(linesBlurB, linesB, saturate(dofBlur));

  float4 tmp = (1 + time.w) * float4(tmpR.x, tmpG.y, tmpB.z, tmpR.w);
  float4 fadeTmp = (1 - smoothstep(0, 1, time.y)) * tmp;
  float4 col = backgroundCol + fadeTmp;

  // vignette
  float r = 0.5 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return smoothstep(0, 2, time.x) * r * col;
}


