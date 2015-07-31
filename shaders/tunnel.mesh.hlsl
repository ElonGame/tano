#include "common.hlsl"

cbuffer F : register(b0)
{
  matrix viewProj;
  float time;
};

cbuffer O : register(b1)
{
  matrix objWorld;
};

struct VsMeshIn
{
  float3 pos : SV_Position;
  float3 normal : Normal;
  float2 uv : TexCoord0;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float4 normal_ws : Normal;
  float2 uv : TexCoord0;
  float dist : TexCoord1;
};

//------------------------------------------------------------------------------
float3 FromSpherical(float r, float phi, float theta)
{
  float yProj = r * sin(theta);
  return float3(yProj * cos(phi), r * cos(theta), yProj * sin(phi));
}

//------------------------------------------------------------------------------
float3 ToSpherical(float3 v)
{
  float r = length(v);
//  if (r == 0)
//    return float3(0, 0, 0);

  return float3(r, atan2(v.z, v.x), acos(v.y / r));
}


//------------------------------------------------------------------------------
// entry-point: vs
VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;

  float4 pos = mul(float4(v.pos/10, 1), objWorld);
  float3 ofs;
  ofs.x = 20 * sin(time);
  ofs.y = 20 * cos(time);
  ofs.z = 20 * sin(time/2) * cos(time/3);
  pos.xyz += ofs;

  if (false)
  {
    float dist = length(pos);
    float3 s = ToSpherical(pos.xyz);
    float fr = 1;
    float f0 = dist / ((0.9 + 0.5 * sin(time) * cos(time/2)) * 20);
    float f1 = (0.9 + 0.5 * sin(time)) * dist / 100;
    float3 posNew = FromSpherical(s.x * fr, s.y * f0, s.z * f1);
    res.pos = mul(float4(posNew, 1), viewProj);    
  }
  else
  {
    float dist = length(pos.xyz - ofs);
    float4x4 mtxX = RotateX(sin(time - dist / 10));
    float4x4 mtxY = RotateY(sin(time - dist / 10));
    float4x4 mtxZ = RotateZ(sin(time - dist / 10));

    float4x4 mtxRot = mul(mtxX, mtxY);
    mtxRot = mul(mtxRot, mtxZ);
    float4x4 m = mul(mtxRot, viewProj);
    res.pos = mul(pos, m);    
  }
  res.normal_ws = mul(float4(v.normal, 0), objWorld);
  res.uv = v.uv;
  res.dist = length(v.pos);
  return res;
}

// entry-point: ps
float4 PsMesh(VsMeshOut p) : SV_Target
{
  return saturate(dot(p.normal_ws.xyz, normalize(float3(0, 1, -1))));
  return 1;
}
