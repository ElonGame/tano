#include "common.hlsl"

cbuffer V : register(b0)
{
  matrix viewProj;
  matrix objWorld;
};

cbuffer P : register(b0)
{
  float4 params;
};

struct VsMeshIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float3 ws_pos : Position;
  float3 normal : Normal;
};

//------------------------------------------------------------------------------
// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;

  float4 pos = mul(float4(v.pos, 1), objWorld);
  res.ws_pos = pos.xyz;
  res.pos = mul(pos, viewProj);
  res.normal = v.normal;
  return res; 
}

// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  float fade = params.x;
  fade += smoothstep(0.1, 1, params.y);
  float3 n = normalize(p.normal);

  float3 l = float3(0, 0, -1);
  float3 v = normalize(float3(0,0,-200) - p.ws_pos);
  float3 r = reflect(l, n);
  float diffuse = saturate(dot(n, l));
  float specular = pow(saturate(dot(v, r)), 10);

  return fade * (diffuse + 0.5 * specular);
}

