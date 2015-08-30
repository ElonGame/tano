#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix view;
  matrix viewProj;
};

cbuffer O : register(b1)
{
  matrix objWorld;
  float4 col;
};

struct VsMeshIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
  float depthVS : DepthVS;
};

struct PSOut
{
  float4 col : SV_Target0;
  float4 revealage : SV_Target1;
};

//------------------------------------------------------------------------------
// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;
  float4x4 worldViewProj = mul(objWorld, viewProj);
  float3 posWS = mul(float4(v.pos, 1), objWorld).xyz;
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(v.normal, (float3x3)objWorld);
  res.depthVS = mul(float4(posWS, 1), view).z;
  return res; 
}

// entry-point: ps
PSOut PsMesh(VsMeshOut p)
{
  PSOut res;
  float a = col.a;
  float z = p.depthVS;
  //float w = pow(a + 0.01f, 4.0f) + max(0.01f, min(1000.0f, 100 / (0.00001f + pow(z / 5.0f, 2.0f) + pow(z / 200.0f, 6.0f))));
  //float w = clamp(pow(min(1.0, a * 10.0) + 0.01, 3.0) * 1e8 * pow(1.0 - p.depthVS * 0.9, 3.0), 1e-2, 3e3);
  float w = pow(a, 1.0) * clamp(0.3 / (1e-5 + pow(z / 200, 4.0)), 1e-2, 3e3);

  float3 n = normalize(p.normal);
  float diffuse = saturate(dot(n, float3(0, 0.5, -0.5)));
  res.col = float4(w * diffuse * col.rgb, w * a);
  res.revealage = col.a;
  return res;
}
