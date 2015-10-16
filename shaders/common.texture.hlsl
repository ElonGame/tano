#include "common.hlsl"

cbuffer P : register(b0)
{
  float brightness;
};

//------------------------------------------------------
struct VSTextureIn
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

struct VSTextureOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};

//------------------------------------------------------
// entry-point: vs
VSTextureOut VsRenderTexture(VSTextureIn v)
{
  return v;
}

//------------------------------------------------------
// entry-point: ps
float4 PsRenderTexture(VSTextureOut p) : SV_Target
{
  return Texture0.Sample(LinearSampler, p.uv) * brightness;
}
