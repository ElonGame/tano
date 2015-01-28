#include "common.hlsl"

cbuffer PerFrame : register(b0)
{
  float2 dim;
};

//------------------------------------------------------
Texture2D Texture0 : register(t0);
sampler PointSampler : register(s0);

//------------------------------------------------------
cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix viewProj;
};

//------------------------------------------------------
float sdSphere(float3 p, float s)
{
  return length(p)-s;
} 

//------------------------------------------------------
float DistanceEstimator(float3 p)
{
  return sdSphere(p, 10);
}


//------------------------------------------------------
float trace(float3 from, float3 direction)
{
  int MaximumRaySteps = 50;
  float MinimumDistance = 0.1f;

  float totalDistance = 0.0;
  int steps;
  for (steps=0; steps < MaximumRaySteps; steps++)
  {
    float3 p = from + totalDistance * direction;
    float distance = DistanceEstimator(p);
    totalDistance += distance;
    if (distance < MinimumDistance)
      break;
  }

  return 1.0-float(steps)/float(MaximumRaySteps);
}
//------------------------------------------------------
float4 PsRaymarcher(VSQuadOut input) : SV_TARGET
{
  float3 cameraPos = float3(0, 0, -100);
  float3 lookAt = float3(0,0,0);

  // calc camera base
  float3 dir = normalize(lookAt - cameraPos);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);
  up = cross(right, dir);

  // calc ray dir 
  // this is done by determining which pixel on the image
  // plane the current pixel represents
  float aspectRatio = dim.x / dim.y;
  float3 r = 2 * (-0.5 + input.pos.x / dim.x) * right + (2 * (-0.5 + input.pos.y / dim.y)) * up + dir;
  r.y /= aspectRatio;
  r = normalize(r);

  return trace(cameraPos, r);
}
