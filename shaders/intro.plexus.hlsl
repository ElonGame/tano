#include "common.hlsl"

cbuffer PS : register(b0)
{
  float4 lineParams;
};

cbuffer GS : register(b0)
{
  matrix world;
  matrix viewProj;
  float3 cameraPos;
  float4 dim;
};


//------------------------------------------------------
// lines
//------------------------------------------------------
struct VsLinesIn
{
  float3 pos : Position;
};

struct VsLinesOut
{
  float3 pos : Position;
};

struct PsLinesIn
{
  float4 pos  : SV_Position;
  // These values represent a point and direction in screen space, so
  // we don't want them perspectively interpolated
  noperspective float4 posDir: TEXCOORD0;
  noperspective float4 endPoints: TEXCOORD1;
};

// entry-point: vs
VsLinesOut VsLines(VsLinesIn v)
{
  VsLinesOut res;
  res.pos = v.pos;

  return res;
}

float2 projToWindow(in float4 pos)
{
    return float2(  dim.x*0.5*((pos.x/pos.w) + 1) + dim.z,
                    dim.y*0.5*(1-(pos.y/pos.w)) + dim.w );
}

[maxvertexcount(4)]
// entry-point: gs
void GsLines(line VsLinesOut input[2], inout TriangleStream<PsLinesIn> outStream)
{
  matrix worldViewProj = mul(world, viewProj);
  float3 pos0 = input[0].pos;
  float3 pos1 = input[1].pos;

  float4 pos0Proj = mul(float4(pos0, 1), worldViewProj);
  float4 pos1Proj = mul(float4(pos1, 1), worldViewProj);

  // Transform position to window space
  float2 points[2];
  points[0] = projToWindow(pos0Proj);
  points[1] = projToWindow(pos1Proj);

  float2 dirs[2];
  dirs[0] = normalize(points[1] - points[0]);
  dirs[1] = normalize(points[0] - points[1]);

  // Triangle strip order
  // 1--3
  // |  |
  // 0--2

  float3 dir, right, up;

  PsLinesIn p;
  float s = 5.5;

  dir = normalize(cameraPos - pos0);
  right = cross(dir, float3(0,1,0));
  right = normalize(pos1-pos0);
  up = cross(right, dir);
  float3 p0 = float3(pos0 - s * right - s * up);
  float3 p1 = float3(pos0 - s * right + s * up);
  float3 p2 = float3(pos1 + s * right - s * up);
  float3 p3 = float3(pos1 + s * right + s * up);

  PsLinesIn output;
  output.endPoints.xy = points[0];
  output.endPoints.zw = points[1];
  output.pos = mul(float4(p0, 1), worldViewProj);
  output.posDir.xy = points[0];
  output.posDir.zw = dirs[0];
  outStream.Append(output);

  output.pos = mul(float4(p1, 1), worldViewProj);
  output.posDir.xy = points[0];
  output.posDir.zw = dirs[0];
  outStream.Append(output);

  output.pos = mul(float4(p2, 1), worldViewProj);
  output.posDir.xy = points[1];
  output.posDir.zw = dirs[1];
  outStream.Append(output);

  output.pos = mul(float4(p3, 1), worldViewProj);
  output.posDir.xy = points[1];
  output.posDir.zw = dirs[1];
  outStream.Append(output);

  outStream.RestartStrip();
}


static float4 WireColor = float4(0.3, 0.3, 0.7, 1);

// entry-point: ps
float4 PsLines(PsLinesIn p) : SV_Target
{
  float2 a = p.endPoints.xy;
  float2 b = p.endPoints.zw;
  float2 pt = p.pos.xy;
  float distAB = distance(a, b);
  float2 v = normalize(b-a);
  // calc projection of point on b-a
  float2 projPt;
  float tt = dot(v, pt-a);
  if (tt > distAB)
    projPt = b;
  else if (tt > 0)
    projPt = a + tt * v;
  else
    projPt = a;
  float dist = distance(pt, projPt);

  float3 params = lineParams.xyz;
  // fade comes from the text fade-in/fade-out settings
  float fade = lineParams.w;

  float t = smoothstep(0, params.x, dist);
  float alpha = 1 - pow(t, params.y);
  alpha *= 1-smoothstep(0, params.z, distAB);

  return WireColor * alpha * fade;
}
