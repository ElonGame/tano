#include "common.hlsl"

Texture2D Texture0 : register(t0);
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
  float2 tex : TexCoord0;
  float4 ss : TexCoord1;    // screen space coordinates of end point
  float4 a : TexCoord2;
  float4 b : TexCoord3;
  float bad : TexCoord4;
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

void OutputVtx(float4 v, float2 tex, inout TriangleStream<GsLinesOut> stream)
{
  GsLinesOut res;
  res.pos = mul(v, proj);
  res.tex = tex;
  res.ss = float4(0,0,0,0);
  res.bad = 0;
  stream.Append(res);
}

void OutputVtx2(float3 v, float2 tex, inout TriangleStream<GsLinesOut> stream)
{
  GsLinesOut res;
  res.pos = mul(float4(v, 1), viewProj);
  res.tex = tex;
  res.ss = float4(0,0,0,0);
  res.bad = 0;
  stream.Append(res);
}

void OutputVtx3(float3 v, float4 ss, float bad, float4 a, float4 b, inout TriangleStream<GsLinesOut> stream)
{
  GsLinesOut res;
  res.pos = mul(float4(v, 1), viewProj);
  res.tex = float2(0,0);
  res.ss = ss;
  res.bad = bad;
  res.a = a;
  res.b = b;
  stream.Append(res);
}

void OutputQuad(float4 a, float3 dir, float3 up, float2 tex[4], inout TriangleStream<GsLinesOut> stream)
{
  float4 v0 = a;
  v0.xyz += -1 * dir - 1 * up;

  float4 v1 = a;
  v1.xyz += -1 * dir + 1 * up;

  float4 v2 = a;
  v2.xyz += +1 * dir - 1 * up;
  
  float4 v3 = a;
  v3.xyz += +1 * dir + 1 * up;

  OutputVtx(v0, tex[0], stream);
  OutputVtx(v1, tex[1], stream);
  OutputVtx(v2, tex[2], stream);
  OutputVtx(v3, tex[3], stream);
 
}

void OutputQuad2(float4 a, float2 dir, float2 up, float2 tex[4], inout TriangleStream<GsLinesOut> stream)
{
  float4 v0 = a;
  v0.xy += -1 * dir.xy - 1 * up.xy;

  float4 v1 = a;
  v1.xy += -1 * dir.xy + 1 * up.xy;

  float4 v2 = a;
  v2.xy += +1 * dir.xy - 1 * up.xy;
  
  float4 v3 = a;
  v3.xy += +1 * dir.xy + 1 * up.xy;

  OutputVtx(v0, tex[0], stream);
  OutputVtx(v1, tex[1], stream);
  OutputVtx(v2, tex[2], stream);
  OutputVtx(v3, tex[3], stream);
}

void OutputQuad3(float3 a, float3 right, float3 up, float2 tex[4], inout TriangleStream<GsLinesOut> stream)
{
  float3 v0 = a - 1 * right - 1 * up;
  float3 v1 = a - 1 * right + 1 * up;
  float3 v2 = a + 1 * right - 1 * up;
  float3 v3 = a + 1 * right + 1 * up;

  OutputVtx2(v0, tex[0], stream);
  OutputVtx2(v1, tex[1], stream);
  OutputVtx2(v2, tex[2], stream);
  OutputVtx2(v3, tex[3], stream);
}

// Cohen-Sutherland
int ClipCode(float4 a)
{
  int res = 0;
  if (a.y > +a.w) res |= 1;
  if (a.y < -a.w) res |= 2;
  if (a.x > +a.w) res |= 4;
  if (a.x < -a.w) res |= 8;
  return res;
}

[maxvertexcount(8)]
void GsLines(line VsLinesOut input[2], inout TriangleStream<GsLinesOut> stream)
{
  /*
      1--3-------5--7
      |  |       |  |
      0--2-------4--6
  */

  float3 a = input[0].pos;
  float3 b = input[1].pos;
  float3 mid = (a+b)/2;

  float3 right = normalize(b-a);
  //float3 dir = float3(0,0,-1); //normalize(camPos.xyz-a);
  float3 dir = normalize(camPos.xyz-mid);
  float3 up = normalize(cross(right, dir));

  float h = 15;
  float w = 20;
  float3 v0 = a - w * right - h * up;
  float3 v1 = a - w * right + h * up;
  float3 v2 = b + w * right - h * up;
  float3 v3 = b + w * right + h * up;

  // compute screen space end points
  float4 cs_a = mul(float4(a, 1), viewProj);
  float4 cs_b = mul(float4(b, 1), viewProj);
  float bad = cs_a.w * cs_b.w;

  float2 ss_a = cs_a.xy / cs_a.w;
  float2 ss_b = cs_b.xy / cs_b.w;

  float4 ss = float4(ss_a.x, ss_a.y, ss_b.x, ss_b.y);

  cs_a.x = -1; cs_a.y = -1;
  cs_a.z = 0.5; cs_a.w = 1;
  OutputVtx3(v0, ss, bad, cs_a, cs_b, stream);

  cs_a.x = -1; cs_a.y = +1;
  cs_a.z = 0.5; cs_a.w = 0;
  OutputVtx3(v1, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = -1;
  cs_a.z = 0.5; cs_a.w = 1;
  OutputVtx3(v2, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = +1;
  cs_a.z = 0.5; cs_a.w = 0;
  OutputVtx3(v3, ss, bad, cs_a, cs_b, stream);
}

[maxvertexcount(8)]
void GsLines2(line VsLinesOut input[2], inout TriangleStream<GsLinesOut> stream)
{
  /*
      1--3-------5--7
      |  |       |  |
      0--2-------4--6
  */

  float3 a = input[0].pos;
  float3 b = input[1].pos;
  float3 mid = (a+b)/2;

  float3 right = normalize(b-a);
  //float3 dir = float3(0,0,-1); //normalize(camPos.xyz-a);
  float3 dir = normalize(camPos.xyz-mid);
  float3 up = normalize(cross(right, dir));

  float h = 25;
  float w = 5;
  float3 v0 = a - w * right - h * up;
  float3 v1 = a - w * right + h * up;
  float3 v2 = a + w * right - h * up;
  float3 v3 = a + w * right + h * up;

  float3 v4 = b - w * right - h * up;
  float3 v5 = b - w * right + h * up;
  float3 v6 = b + w * right - h * up;
  float3 v7 = b + w * right + h * up;

  // compute screen space end points
  float4 cs_a = mul(float4(a, 1), viewProj);
  float4 cs_b = mul(float4(b, 1), viewProj);
  float bad = cs_a.w * cs_b.w;

  float2 ss_a = cs_a.xy / cs_a.w;
  float2 ss_b = cs_b.xy / cs_b.w;

  float4 ss = float4(ss_a.x, ss_a.y, ss_b.x, ss_b.y);

  cs_a.x = -1; cs_a.y = -1;
  cs_a.z = 0.0; cs_a.w = 1;
  OutputVtx3(v0, ss, bad, cs_a, cs_b, stream);

  cs_a.x = -1; cs_a.y = +1;
  cs_a.z = 0.0; cs_a.w = 0;
  OutputVtx3(v1, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = -1;
  cs_a.z = 0.5; cs_a.w = 1;
  OutputVtx3(v2, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = +1;
  cs_a.z = 0.5; cs_a.w = 0;
  OutputVtx3(v3, ss, bad, cs_a, cs_b, stream);


  cs_a.x = -1; cs_a.y = -1;
  cs_a.z = 0.5; cs_a.w = 1;
  OutputVtx3(v4, ss, bad, cs_a, cs_b, stream);

  cs_a.x = -1; cs_a.y = +1;
  cs_a.z = 0.5; cs_a.w = 0;
  OutputVtx3(v5, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = -1;
  cs_a.z = 1.0; cs_a.w = 1;
  OutputVtx3(v6, ss, bad, cs_a, cs_b, stream);

  cs_a.x = +1; cs_a.y = +1;
  cs_a.z = 1.0; cs_a.w = 0;
  OutputVtx3(v7, ss, bad, cs_a, cs_b, stream);

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

float line_distance3(float3 p, float3 a, float3 b)
{
    float dist = distance(a,b);
    float3 v = normalize(b-a);
    float t = dot(v,p-a);
    float3 spinePoint;
    if (t > dist) spinePoint = b;
    else if (t > 0.0) spinePoint = a + t*v;
    else spinePoint = a;
    return distance(p,spinePoint);
}

float4 PsLines(GsLinesOut input) : Sv_Target
{
  //return 1;
  float3 col = Texture0.Sample(PointSampler, input.a.zw).rgb;
  float aa = length(col);
  float bb = pow(aa, 5);
  return float4(bb, bb, bb, 0);

  return 1 - length(input.a.xy);
  float xxx = input.a.x;
  return float4(xxx, xxx, xxx, 1);
  return input.a.x;
  float2 ndc = 2 * (input.pos.xy / dim.xy - 0.5);
  ndc.y *= -1;

  float dd = 1 - length(ndc - input.a.xy);
  return float4(dd, dd, dd, 1);
  return length(input.pos.xy - input.a.xy);
  //if (input.bad < 0)
   // return 1;
  // compute ndc pixel coords
  //float2 ndc = 2 * (input.pos.xy / dim.xy - 0.5);
  //ndc.y *= -1;
  float3 xx = float3(ndc.x * input.a.w, ndc.y * input.a.w, input.pos.z * input.a.w);
  float d = saturate(1 - 10 * line_distance3(xx, input.a.xyz, input.b.xyz));
  //float d = saturate(1 - 10 * line_distance(ndc, input.a.xy, input.b.xy));
  d = pow(d, 20);
  return float4(float3(0.2, 0.2, 0.8) * d, 0);
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
