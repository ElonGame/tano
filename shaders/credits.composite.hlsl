#include "common.hlsl"

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 col = Texture0.Sample(PointSampler, uv); 

  float exposure = tonemap.x;
  float minWhite = tonemap.y;

  float f = saturate(smoothstep(0, 1, 1 - 1.5 * uv.y));
  col = f * col;

  // vignette
  float r = 0.8 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));

  col = ToneMapReinhard(col, exposure, minWhite);

  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  return r * col;
}
