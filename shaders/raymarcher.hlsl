// distance functions from iq: http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm

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
float sdBox( float3 p, float3 b )
{
  float3 d = abs(p) - b;
  return min(max(d.x,max(d.y,d.z)),0.0) + length(max(d,0.0));
}

//------------------------------------------------------
float sdTorus( float3 p, float2 t )
{
  float2 q = float2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

//------------------------------------------------------
float distScene(float3 p)
{
  return sdTorus(p, float2(2, 0.5));
  return 
    min(
      min(sdTorus(p, float2(1, 1)), sdSphere(p, 2)), 
      sdBox(p, float3(2,1,1)));
}

//------------------------------------------------------
float3 getNormal(float3 p)
{
  float h = 0.0001f;

  return normalize(float3(
    distScene(p + float3(h, 0, 0)) - distScene(p - float3(h, 0, 0)),
    distScene(p + float3(0, h, 0)) - distScene(p - float3(0, h, 0)),
    distScene(p + float3(0, 0, h)) - distScene(p - float3(0, 0, h))));
}

//------------------------------------------------------
float trace(float3 from, float3 direction)
{
  int MaximumRaySteps = 10;
  float MinimumDistance = 0.1f;

  float totalDistance = 0.0;
  float col = 0;
  int steps;
  for (steps=0; steps < MaximumRaySteps; steps++)
  {
    float3 p = from + totalDistance * direction;
    float distance = distScene(p);
    totalDistance += distance;
    if (distance < MinimumDistance)
    {
      return 0.1f + dot(getNormal(p), normalize(float3(0, 1, -1)));
      col = float(steps)/float(MaximumRaySteps);
      break;
    }
  }

  return col;
}
//------------------------------------------------------
float4 PsRaymarcher(VSQuadOut input) : SV_TARGET
{
  float3 cameraPos = float3(0, 0, -100);
  float3 lookAt = float3(0,0,0);
  float f = 5;

  // calc camera base
  float3 dir = normalize(lookAt - cameraPos);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);
  up = cross(right, dir);

  // calc ray dir 
  // this is done by determining which pixel on the image
  // plane the current pixel represents
  float aspectRatio = dim.x / dim.y;
  float3 r = 2 * (-0.5 + input.pos.x / dim.x) * right + (2 * (-0.5 + input.pos.y / dim.y)) * up + f * dir;
  r.y /= aspectRatio;
  r = normalize(r);

  return trace(cameraPos, r);
}
