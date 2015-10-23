#include "common.hlsl"

//------------------------------------------------------
struct VSTextIn
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

struct VSTextOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

cbuffer P : register(b0)
{
  float time;
  float brightness;
};

//------------------------------------------------------
// entry-point: vs
VSTextOut VsText(VSTextIn v)
{
  return v;
}

//------------------------------------------------------
// entry-point: ps
float4 PsText(VSQuadOut p) : SV_Target
{
  float4 tex0 = Texture0.Sample(LinearSampler, p.uv);
  float4 tex1 = Texture0.Sample(LinearSampler, p.uv);
  return tex0 * brightness;
}


