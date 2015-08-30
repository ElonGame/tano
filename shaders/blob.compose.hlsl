#include "common.hlsl"

cbuffer F : register(b0)
{
  float2 tonemap; // x = exposure/lumAvg, y = min_white
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;

  float4 orgCol = Texture0.Sample(PointSampler, uv);
  float opacity = Texture1.Sample(PointSampler, uv).x;

  float4 col = float4(orgCol.rgb / clamp(orgCol.a, 1e-4, 5e4), opacity);

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);
  
  //  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  return col;
}
