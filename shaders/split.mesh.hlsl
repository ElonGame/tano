#include "common.hlsl"
#include "split.common.hlsl"
#include "noise_lib.hlsl"

cbuffer V : register(b0)
{
  matrix view;
  matrix viewProj;
};

cbuffer P : register(b0)
{
  float3 cameraPos;
};

cbuffer O : register(b1)
{
  matrix objWorld;
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

  float4 pos = mul(float4(v.pos, 1), objWorld);
  res.pos = mul(pos, viewProj);
  res.ws_pos = pos.xyz;
  res.normal = v.normal;
  res.depthVS = mul(float4(pos.xyz, 1), view).z;

  return res; 
}

// entry-point: ps
PSOut PsMeshTrans(VsMeshOut p)
{
  PSOut res;
  float a = 0.7;
  float z = p.depthVS;
  float w = pow(a, 1.0) * clamp(0.3 / (1e-5 + pow(z / 200, 4.0)), 1e-2, 3e3);

  float3 v = normalize(cameraPos - p.ws_pos);
  float3 l = SUN_DIR;

  float3 pp = p.ws_pos;
  float3 n = normalize(p.normal);

  float3 r = reflect(l, n);
  float diffuse = saturate(dot(n, l));
  float specular = pow(saturate(dot(v, r)), 10);

  //return specular;
  float4 final = 0.2 * diffuse * float4(0.2, 0.2, 0.8, 1) + 0.3 * specular * float4(1,1,1,1);
  res.col = w * float4(a * final.rgb, a);
  res.revealage = a;
  return res;
}


// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  float3 v = normalize(cameraPos - p.ws_pos);
  float3 l = SUN_DIR;

  float3 pp = p.ws_pos;
  //float org = simplexNoise(pp);
  //return abs(p.ws_pos.z / 100);
  //return org;
  // float eps = 0.001f;
  // float dx = noise(pp + float3(eps, 0, 0)) - noise(pp + float3(-eps, 0, 0));
  // float dy = noise(pp + float3(0, eps, 0)) - noise(pp + float3(0, -eps, 0));
  // float dz = noise(pp + float3(0, 0, eps)) - noise(pp + float3(0, 0, -eps));

  // float3 modNormal = normalize(float3(dx, dy, dz));
  //return float4(modNormal, 1);
  float3 n = normalize(p.normal);
  //float n = lerp(orgNormal, modNormal, saturate(dot(orgNormal, v)));
  // float3 n = normalize(p.normal - 0.25 * modNormal);
  //float3 n = normalize(p.normal);

  float3 r = reflect(l, n);
  float diffuse = saturate(dot(n, l));
  float specular = pow(saturate(dot(v, r)), 10);

  //return specular;

  return 0.2 * diffuse * float4(0.2, 0.2, 0.8, 1) + 0.3 * specular * float4(1,1,1,1);
}
