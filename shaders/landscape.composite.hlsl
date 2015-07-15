#include "common.hlsl"

cbuffer F : register(b0)
{
  float4 tonemap; // x = shoulder, y = max_white, z = exposure/lumAvg, w = min_white
};

float4 PsComposite(VSQuadOut p) : SV_Target
{
  // Texture0 : color + bloom
  // Texture1 : color + bloom blurred
  // Texture2 : lensflare
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

  //return Texture2.Sample(LinearSampler, uv);

  float4 col = 
    Texture0.Sample(PointSampler, uv) + 
    Texture1.Sample(PointSampler, uv) +
    Texture2.Sample(LinearSampler, uv);

  float exposure = tonemap.z;
  float minWhite = tonemap.w;

  col = ToneMapReinhard(col, exposure, minWhite);
  
   // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * col;
}
