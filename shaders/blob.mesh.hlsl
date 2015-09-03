#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix view;
  matrix viewProj;
  float3 cameraPos;
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
  float3 posWS : Position;
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
  res.posWS = posWS;
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

  float3 lightPos = float3(0, 0, -500);

  float3 n = normalize(p.normal);
  float3 l = normalize(lightPos - p.posWS);
  float3 v = normalize(cameraPos - p.posWS);
  float3 r = reflect(l, n);
  float specular = pow(saturate(dot(v, r)), 100);
  float diffuse = saturate(0.5 * dot(n, l));
  float ambient = 0.1;
  res.col = float4(w * ((ambient + diffuse) * col.rgb + specular * float3(1,1,1)), w * a);
  res.revealage = col.a;
  return res;
}
