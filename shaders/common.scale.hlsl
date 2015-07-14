#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 scaleBias; // x = scale, y = bias
};

//------------------------------------------------------
float4 PsScaleBias(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(LinearSampler, uv);
  return max(float4(0,0,0,0), col - scaleBias.y) * scaleBias.x;
}

//------------------------------------------------------
float4 PsScaleBiasSecondary(VSQuadOut p) : SV_Target
{
  
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(LinearSampler, uv);
  float v = Texture1.Sample(LinearSampler, uv).a;
  return max(float4(0,0,0,0), v - scaleBias.y) * scaleBias.x * col;
}
