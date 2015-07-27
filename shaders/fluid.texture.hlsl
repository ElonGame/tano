#include "common.hlsl"

struct VsTextureIn
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

struct VsTextureOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

VsTextureOut VsMain(VsTextureIn v)
{
	VsTextureOut res;
	res.pos = v.pos;
	res.uv = v.uv;
	return res;
}

float4 PsMain(VsTextureOut p) : SV_Target
{
  return Texture0.Sample(LinearSampler, p.uv.xy);
}
