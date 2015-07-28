// basic shader variants

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
};

struct VsPosIn
{
  float3 pos : Position;
};

struct VsPosColorIn
{
  float3 pos : Position;
  float4 color : Color;
};

struct VsPosNormalIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsPosOut
{
  float4 pos : SV_Position;
};

struct VsPosColorOut
{
  float4 pos : SV_Position;
  float4 color : Color;
};

struct VsPosNormalOut
{
  float4 pos : SV_Position;
  float3 wsPos : Position;
  float3 wsNormal : Normal;
};

static float3 LIGHT_POS = float3(0, 100, -100);

//------------------------------------------------------
// pos
//------------------------------------------------------

// entry-point: vs
VsPosOut VsPos(VsPosIn v)
{
  VsPosOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  return res;
}

// entry-point: ps
float4 PsPos(VsPosOut p) : SV_Target
{
  return 1;
}

//------------------------------------------------------
// pos-color
//------------------------------------------------------

// entry-point: vs
VsPosColorOut VsPosColor(VsPosColorIn v)
{
  VsPosColorOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.color = v.color;
  return res;
}

// entry-point: ps
float4 PsPosColor(VsPosColorOut p) : SV_Target
{
  return p.color;
}


//------------------------------------------------------
// pos-normal
//------------------------------------------------------

// entry-point: vs
VsPosNormalOut VsPosNormal(VsPosNormalIn v)
{
  VsPosNormalOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.wsPos = mul(float4(v.pos, 1), world).xyz;
  res.wsNormal = mul(float4(v.normal, 0), world).xyz;
  return res;
}

// entry-point: ps
float4 PsPosNormal(VsPosNormalOut p) : SV_Target
{
  // pos -> light
  float3 ll = normalize(LIGHT_POS - p.wsPos);
  return saturate(dot(ll, p.wsNormal));
}

