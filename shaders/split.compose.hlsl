#include "common.hlsl"

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
};

// crazy-trippy sampling :)
float4 sampling(VSQuadOut p)
{
  float2 uv = p.uv.xy;
  // -1..1
  float2 xx = -1 + 2 * uv;
  float2 dd = normalize(float2(0.5, 0.5) - uv);

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float blocker = Texture1.Sample(PointSampler, uv).r;
  float4 orgCol = Texture2.Sample(PointSampler, uv);
  float opacity = Texture3.Sample(PointSampler, uv).x;

  float4 glow = float4(0,0,0,0);
  float2 uvStep = uv;
  float scale = 0.7;
  for (int i = 0; i < 10; ++i)
  {
      float4 backgroundCol = Texture0.Sample(PointSampler, uvStep);
      float blocker = Texture2.Sample(PointSampler, uvStep).r;

      uvStep += 0.1 * dd;
      glow += scale * (1-blocker) * backgroundCol;
      scale *= scale;
  }

  return glow;

}

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // 0..1
  float2 uv = p.uv.xy;
  // -1..1
  float2 xx = -1 + 2 * uv;
  float2 dd = normalize(float2(0.5, 0.5) - uv);

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float blocker = Texture1.Sample(PointSampler, uv).r;
  float4 orgCol = Texture2.Sample(PointSampler, uv);
  float opacity = Texture3.Sample(PointSampler, uv).x;

  float4 glow = float4(0,0,0,0);
  float2 uvStep = uv;
  float scale = 0.4;
  for (int i = 0; i < 10; ++i)
  {
      float4 backgroundCol = Texture0.Sample(PointSampler, uvStep);
      float blocker = Texture1.Sample(PointSampler, uvStep).r;

      uvStep += 0.005 * dd;
      glow += scale * (1-blocker) * backgroundCol;
      scale *= 0.65;
  }

  float4 col = 
    backgroundCol +
    float4(orgCol.rgb / clamp(orgCol.a, 1e-4, 5e4), opacity) + 
    glow;

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
