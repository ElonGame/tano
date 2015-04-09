#include "common.hlsl"

Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
sampler PointSampler : register(s0);
sampler LinearSampler : register(s1);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
  float3 cameraUp;
};

struct VsLinesIn
{
  float3 pos0 : Position0;
  float3 pos1 : Position1;
  float4 weights : TexCoord0;
  float radius : TexCoord1;
  float aspect : TexCoord2;
};

struct VsLinesOut
{
  float4 pos : SV_Position;
  float4 tex : Texture0;
};


//
// Vertex shader for AA lines
//
VsLinesOut VsMain(VsLinesIn input)
{
  VsLinesOut output;

  float4x4 worldViewProj = mul(world, viewProj);
  float aspect = input.aspect;
  float radius = input.radius;
  float4 weights = input.weights;

    // Transform the input points.
  float4 p0 = mul(float4(input.pos0, 1.0f), worldViewProj);
  float4 p1 = mul(float4(input.pos1, 1.0f), worldViewProj);

    // Warp transformed points by aspect ratio.
  float4 w0 = p0;
  float4 w1 = p1;
  w0.y /= aspect;
  w1.y /= aspect;

    // Calc vectors between points in screen space.
  float2 delta2 = w1.xy / w1.z - w0.xy / w0.z;
  float3 delta_p;

  delta_p.xy = delta2;
  delta_p.z = w1.z - w0.z;

    //
    // Calc UV basis vectors.
    //
    // Calc U
  float  len = length(delta2);
  float3 U = delta_p / len;

    // Create V orthogonal to U.
  float3 V;
  V.x = U.y;
  V.y = -U.x;
  V.z = 0;

    // Calculate output position based on this
    // vertex's weights.
  output.pos = p0 * weights.x + p1 * weights.y;

    // Calc offset part of postion.
  float3 offset = U * weights.z + V * weights.w;

    // Apply line thickness.
  offset.xy *= radius;

    // Unwarp by inverse of aspect ratio.
  offset.y *= aspect;

    // Undo perspective divide since the hardware will do it.
  output.pos.xy += offset.xy * output.pos.z;

    // Set up UVs.  We have to use projected sampling rather
    // than regular sampling because we don't want to get
    // perspective correction.
  output.tex.x = weights.z;
  output.tex.y = weights.w;
  output.tex.z = 0.0f;
  output.tex.w = 1.0f;

  return output;
}

float4 PsMain(VsLinesOut input) : SV_Target
{
  float2 uv = input.tex.xy / input.tex.w;
  float4 col = Texture0.Sample(LinearSampler, uv);
  return col;
}
