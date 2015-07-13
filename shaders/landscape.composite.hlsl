#include "common.hlsl"

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 time;
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
  float3 cameraUp;
  float4 nearFar : NEAR_FAR;
  float4 tonemap; // x = shoulder, y = max_white, z = exposure/lumAvg, w = min_white
};

float4 PsComposite(VSQuadOut p) : SV_Target
{
  // Texture0 : color + bloom
  // Texture1 : color + bloom blurred
  // Texture2 : lensflare
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

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
