#include "common.hlsl"

Texture2D Texture0 : register(t0);
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
  float4 cook;
  float4 diffuse;
  //float roughness_value;
  //float ref_at_norm_incidence;
};

struct VsMeshIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsMeshOut
{
  float4 pos : SV_Position;
  float3 pos_world : Position;
  float3 normal : Normal;
};

//------------------------------------------------------
float4 cook_torrance(in float3 normal, in float3 viewer, in float3 light)
{    
    float roughness_value = cook.x;
    float ref_at_norm_incidence = cook.y;
    // Compute any aliases and intermediary values
    // -------------------------------------------
    float3 half_vector = normalize( light + viewer );
    float NdotL        = saturate( dot( normal, light ) );
    float NdotH        = saturate( dot( normal, half_vector ) );
    float NdotV        = saturate( dot( normal, viewer ) );
    float VdotH        = saturate( dot( viewer, half_vector ) );
    float r_sq         = roughness_value * roughness_value;
 
    // Evaluate the geometric term (Cook-Torrance)
    // --------------------------------
    float geo_numerator   = 2.0f * NdotH;
    float geo_denominator = VdotH;
 
    float geo_b = (geo_numerator * NdotV ) / geo_denominator;
    float geo_c = (geo_numerator * NdotL ) / geo_denominator;
    float geo   = min( 1.0f, min( geo_b, geo_c ) );
 
 
    // Now evaluate the roughness term (Beckmann)
    // -------------------------------
    float roughness_a = 1.0f / ( 4.0f * r_sq * pow( NdotH, 4 ) );
    float roughness_b = NdotH * NdotH - 1.0f;
    float roughness_c = r_sq * NdotH * NdotH;

    float roughness = roughness_a * exp( roughness_b / roughness_c );
 
    // Next evaluate the Fresnel value (Schlick)
    // -------------------------------
    float fresnel = pow( 1.0f - VdotH, 5.0f );
    fresnel *= ( 1.0f - ref_at_norm_incidence );
    fresnel += ref_at_norm_incidence;
 
 
    // Put all the terms together to compute
    // the specular term in the equation
    // -------------------------------------
    float3 Rs_numerator   = ( fresnel * geo * roughness );
    float Rs_denominator  = NdotV * NdotL;
    float3 Rs             = Rs_numerator/ Rs_denominator;
 
 
 
    // Put all the parts together to generate
    // the final colour
    // --------------------------------------
    float3 cSpecular = float3(1,1,1);
    float3 cDiffuse = float3(0.2, 0.2, 0.2);
    float3 final = max(0.0f, NdotL) * (cSpecular * Rs + cDiffuse);
  
    // Return the result
    // -----------------
    return float4( final, 1.0f );
}

//------------------------------------------------------
// mesh
//------------------------------------------------------

VsMeshOut VsMesh(VsMeshIn v)
{
  VsMeshOut res;
  matrix worldViewProj = mul(world, viewProj);
  res.pos = mul(float4(v.pos, 1), worldViewProj);
  res.pos_world = mul(float4(v.pos, 1), world).xyz;
  res.normal = mul(float4(v.normal, 0), world).xyz;
  return res;
}

float4 PsMesh(VsMeshOut p) : SV_Target
{
  //float3 LIGHT_POS = float3(0, 100, 0);
  float3 normal = normalize(p.normal);
  float3 viewer = normalize(cameraPos - p.pos_world);
  //float3 light = normalize(LIGHT_POS - p.pos_world);

  //float4 col = DIFFUSE * cook_torrance(normal, viewer, viewer);

//  return float4(normal, 1);
  float col = dot(normal, viewer);

  return pow(saturate(col * diffuse), 1.0/2.2);
}
