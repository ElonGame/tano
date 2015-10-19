#include "common.hlsl"

cbuffer F : register(b0)
{
  float2 tonemap; // x = exposure/lumAvg, y = min_white
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // Texture0: color
  // Texture1: lines
  // Texture2: blur
  float2 uv = p.uv.xy;

  float4 col = Texture0.Sample(LinearSampler, uv);
  float4 lines = Texture1.Sample(LinearSampler, uv);
  float4 blur = Texture2.Sample(LinearSampler, uv); 

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col.rgb = 0.8 * col.rgb;
  col.rgb += pow(abs(col.a * float3(1, 1, 1.5) * blur.rgb), 1.5);
  col.rgb += lines.rgb;

  col = ToneMapReinhard(col, exposure, minWhite);
  
   // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float2 xx = -1 + 2 * uv;
  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * col;
}
