#include "common.hlsl"
#include "split.common.hlsl"

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
  float2 lightPos;
  float3 camDir;
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // 0..1
  float2 uv = p.uv.xy;
  // -1..1
  float2 xx = -1 + 2 * uv;

  float4 backgroundCol = Texture0.Sample(LinearSampler, uv);
  float4 orgCol = Texture1.Sample(LinearSampler, uv);
  float opacity = Texture2.Sample(LinearSampler, uv).x;

  float ss = smoothstep(0, 1, dot(camDir, -SUN_DIR));

  float4 col = backgroundCol + float4(orgCol.rgb / clamp(orgCol.a, 1e-4, 5e4), opacity);

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);
  
  //  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  // vignette
  float r = 0.8 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  col = r * col;

  return col;
}
