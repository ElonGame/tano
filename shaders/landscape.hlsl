#include "common.hlsl"

Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
sampler PointSampler : register(s0);

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
  float3 cameraUp;
};

struct VsBoidsIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsBoidsOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
};

struct VsLandscapeIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsLandscapeOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
  float3 rayDir : Texture0;
  float distance : TexCoord1;
};

static float4 BOID_COLOR = float4(0.4, 0.2, 0.2, 1);
static float3 FOG_COLOR = 0.5 * float3(0.5, 0.6, 0.7);
static float3 SUN_COLOR = 0.5 * float3(1.5, 0.9, 0.3);
static float3 SUN_DIR = normalize(float3(-0.5, -0.2, -0.4));
//static float3 SUN_DIR = normalize(float3(0, -0.2, -1));

//------------------------------------------------------
// sky
//------------------------------------------------------

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, 20));
  return fogColor;
}

float4 PsSky(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 r = -1 + 2 * uv;

  float f = 1;

  // calc camera base
  float3 dir = normalize(cameraLookAt - cameraPos);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);
  up = cross(right, dir);

  // calc ray dir 
  // this is done by determining which pixel on the image
  // plane the current pixel represents
  float aspectRatio = dim.x / dim.y;
  float3 rayDir = 2 * (-0.5 + p.pos.x / dim.x) * right + (2 * (-0.5 + p.pos.y / dim.y)) * up + f * dir;
  rayDir.y /= aspectRatio;
  rayDir = normalize(rayDir);

  return float4(FogColor(rayDir), 1);
}

//------------------------------------------------------
// landscape
//------------------------------------------------------

VsLandscapeOut VsLandscape(VsLandscapeIn v)
{
  VsLandscapeOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  float3 dir = v.pos - cameraPos;
  res.distance = length(dir);
  res.rayDir = dir * 1/res.distance;
  return res;
}

float4 PsLandscape(VsLandscapeOut p) : SV_Target
{
  float3 amb = float3(0.05, 0.05, 0.05);
  float dff = saturate(dot(-SUN_DIR, p.normal));
  float3 col = amb + dff * float3(0.1, 0.1, 0.25);

  float b = 0.001;
  float fogAmount = max(0, 1 - exp(-(p.distance - 500) * b));

  float3 fogColor = FogColor(p.rayDir);

  return float4(lerp(col, fogColor, fogAmount), 1);
}

//------------------------------------------------------
// edge detection
//------------------------------------------------------
float4 PsEdgeDetect(VSQuadOut input) : SV_Target
{
  // Sobel filter
  float2 ofs = 1 / dim.xy;
  float3 l00 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, -ofs.y)).rgb;
  float3 l01 = Texture0.Sample(PointSampler, input.uv + float2(     0, -ofs.y)).rgb;
  float3 l02 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, -ofs.y)).rgb;

  float3 l10 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, 0)).rgb;
  float3 l11 = Texture0.Sample(PointSampler, input.uv + float2(     0, 0)).rgb;
  float3 l12 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, 0)).rgb;

  float3 l20 = Texture0.Sample(PointSampler, input.uv + float2(-ofs.x, +ofs.y)).rgb;
  float3 l21 = Texture0.Sample(PointSampler, input.uv + float2(     0, +ofs.y)).rgb;
  float3 l22 = Texture0.Sample(PointSampler, input.uv + float2(+ofs.x, +ofs.y)).rgb;
  
  float3 gx = +1 * l00 -1 * l02 +2 * l10 -2 * l12 +1 * l20 -1 * l22;
  float3 gy = +1 * l00 +2 * l01 +1 * l02 -1 * l20 -2 * l21 -1 * l22;
  
  float3 e = sqrt(gx*gx+gy*gy);
  float t = max(e.x, e.y);
  return t > 0.2 ? 1 : 0;
}

//------------------------------------------------------
// boids
//------------------------------------------------------
VsBoidsOut VsBoids(VsBoidsIn input)
{
  VsBoidsOut output;
  float4x4 mtxWorldViewProj = mul(world, viewProj);
  output.pos = mul(float4(input.pos, 1), mtxWorldViewProj);
  output.normal = mul(float4(input.normal, 0), world).xyz;
  output.normal = input.normal;
  return output;
}

float4 PsBoids(VsBoidsOut p) : SV_Target
{
  return BOID_COLOR * max(0,(dot(normalize(p.normal), float3(0,-1,0))));
}

//------------------------------------------------------
// composite
//------------------------------------------------------
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float4 edge = Texture1.Sample(PointSampler, uv);

   // gamma correction
  float4 color = pow(backgroundCol, 1.0/2.2);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * color;
}

