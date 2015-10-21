#include "common.hlsl"

cbuffer V : register(b0)
{
  matrix view;
  matrix viewProj;
  float time;
};

cbuffer P : register(b0)
{
  float3 cameraPos;
};

struct VsFaceIn
{
  float3 pos : Position;
};

struct VsFaceOut
{
  float4 pos : SV_Position;
  float3 pos_ws : Position0;
  float3 pos_vs : Position1;
};

struct PsFaceIn
{
  float4 pos : SV_Position;
  float3 pos_ws : Position0;
  float3 pos_vs : Position1;
  float3 normal : Normal;
};

//------------------------------------------------------------------------------
// entry-point: vs
VsFaceOut VsFace(VsFaceIn v)
{
  VsFaceOut res;
  res.pos = mul(float4(v.pos, 1), viewProj);
  res.pos_vs = mul(float4(v.pos, 1), view).xyz;
  res.pos_ws = v.pos;
  return res;
}

[maxvertexcount(3)]
// entry-point: gs
void GsFace(triangle VsFaceOut input[3], inout TriangleStream<PsFaceIn> outStream)
{
    PsFaceIn output;

    float3 faceEdgeA = input[1].pos_ws - input[0].pos_ws;
    float3 faceEdgeB = input[2].pos_ws - input[0].pos_ws;

    output.normal = normalize(cross(faceEdgeB, faceEdgeA));
    output.pos = input[0].pos;
    output.pos_ws = input[0].pos_ws;
    output.pos_vs = input[0].pos_vs;
    outStream.Append(output);
 
    output.pos = input[1].pos;
    output.pos_ws = input[1].pos_ws;
    output.pos_vs = input[1].pos_vs;
    outStream.Append(output);

    output.pos = input[2].pos;
    output.pos_ws = input[2].pos_ws;
    output.pos_vs = input[2].pos_vs;
    outStream.Append(output);

    outStream.RestartStrip();
}

// entry-point: ps
float4 PsFace(PsFaceIn p) : SV_Target
{
  float3 n = normalize(p.normal);
  float3 l = normalize(cameraPos - p.pos_ws);

  float diffuse = saturate(dot(n, l));

  float s = saturate(1 - p.pos_vs.z / 2000); 
  return s * float4(diffuse, diffuse, diffuse, diffuse);
}
