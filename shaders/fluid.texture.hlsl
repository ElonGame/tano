#include "common.hlsl"

struct VsTextureIn
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
};

VsTextureIn VsMain(VsTextureIn v)
{
	return v;
}

float4 PsMain(VsTextureIn p) : SV_Target
{
  return Texture0.Sample(LinearSampler, p.uv.xy);
}
