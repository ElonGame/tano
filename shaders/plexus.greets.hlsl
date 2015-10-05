#include "common.hlsl"

static float3 LIGHT_POS = float3(-50, 50, -200);
static float SPECULAR_POWER = 200;

cbuffer V : register(b0)
{
  matrix viewProj;
  matrix objWorld;
};

cbuffer P : register(b0)
{
  float3 camPos;
};

struct VsIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsOut
{
  float4 pos : SV_Position;
  float4 pos_ws : Position;
  float4 normal_ws : Normal;
};

// entry-point: vs
VsOut VsGreets(VsIn v)
{
  VsOut res;
  matrix worldViewProj = mul(objWorld, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.pos_ws = mul(float4(v.pos, 1), objWorld);
  res.normal_ws = mul(float4(v.normal, 0), objWorld);
  return res;
}

// entry-point: ps
float4 PsGreets(VsOut v) : SV_Target
{
  float3 lightDir = normalize(camPos - v.pos_ws.xyz);
  float3 viewDir = normalize(camPos - v.pos_ws.xyz);
  float3 h = normalize(lightDir + viewDir);
  float3 normal = normalize(v.normal_ws.xyz);

  float ambient = 0.2;
  float diffuse = saturate(dot(normal, lightDir));
  float specular = pow(saturate(dot(h, normal)), SPECULAR_POWER);
  return (ambient + diffuse + specular) * float4(0.2, 0.2, 0.1, 1);
}
