// distance functions from iq: http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm

#include "common.hlsl"
#include "raymarcher_lib.hlsl"

cbuffer PerFrame : register(b0)
{
  float2 dim;
  float time;
};

//------------------------------------------------------
cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix viewProj;
};

//------------------------------------------------------
float s2(float3 p)
{
  float t = sin(time / 5 * p.x * p.y);
  float s = 4 + 0.5 * t;
  return sdBox(p, float3(s,s,s)) + 0.15 * t + 0.1 * cos(time * p.z);
}

float opRep( float3 p, float3 c )
{
    float3 q = modf(p,c)-0.5*c;
    return s2( q );
}

//------------------------------------------------------
float distScene(float3 p)
{
  //return sdBox(p, float3(1,1,1));
  //return sdTorus(p, float2(2, 0.5));
//  return s2(p);
  return s2(p);
  float s = 40;
  //return opRep(p, float3(s, s, s));
  return min(s2(p), sdBox(p, float3(2,1,1)));
}

//------------------------------------------------------
float3 getNormal(float3 p)
{
  float3 eps = float3(0.01, 0.0, 0.0);
  return normalize(float3(
    distScene(p + eps.xyy) - distScene(p - eps.xyy),
    distScene(p + eps.yxy) - distScene(p - eps.yxy),
    distScene(p + eps.yyx) - distScene(p - eps.yyx)));
}

//------------------------------------------------------
float trace(float3 from, float3 direction)
{
  int MaximumRaySteps = 512;
  float MinimumDistance = 0.01f;

  float totalDistance = 0.0;
  float col = 0;
  int steps;
  float prevDistance = 0;
  for (steps=0; steps < MaximumRaySteps; steps++)
  {
    float3 p = from + totalDistance * direction;
    float distance = distScene(p);
    if (distance < MinimumDistance)
    {
      if (distance < 0)
        distance = prevDistance + distance;
      totalDistance += distance;
      p = from + totalDistance * direction;
      return 0.1f + dot(getNormal(p), normalize(from - p));
      col = float(steps)/float(MaximumRaySteps);
      break;
    }
    else
    {
      totalDistance += 0.25 * distance;
    }
    prevDistance = distance;
  }

  return col;
}
//------------------------------------------------------
float4 PsRaymarcher(VSQuadOut input) : SV_TARGET
{
  float ss = 20 * sin(time);
  float cc = 20 * cos(time);
  float3 cameraPos = float3(ss, 3, cc);
  //float3 cameraPos = float3(0, 0, -10);
  float3 lookAt = float3(0,0,0);
  float f = 2;

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

  float t = trace(cameraPos, r);
  return float4(t, 1.1 * t, 1.2 * t, t);
  return trace(cameraPos, r);
}
