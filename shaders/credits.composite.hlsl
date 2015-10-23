#include "common.hlsl"

// Also see:  https://www.shadertoy.com/view/4sfGzS
float iqhash(float n)
{
    return frac(sin(n)*43758.5453);
}

float iqnoise(float3 x)
{
    // The noise function returns a value in the range -1.0f -> 1.0f
    float3 p = floor(x);
    float3 f = frac(x);

    f       = f*f*(3.0-2.0*f);
    float n = p.x + p.y*57.0 + 113.0*p.z;
    return lerp(lerp(lerp( iqhash(n+0.0  ), iqhash(n+1.0  ),f.x),
                   lerp( iqhash(n+57.0 ), iqhash(n+58.0 ),f.x),f.y),
               lerp(lerp( iqhash(n+113.0), iqhash(n+114.0),f.x),
                   lerp( iqhash(n+170.0), iqhash(n+171.0),f.x),f.y),f.z);
}

float NoiseOctaves(float3 x, float s, int iterations)
{
  float res = 0;
  int i;
  for (i = 0; i < iterations; ++i)
  {
    res += s * iqnoise(float3(x.x * 10 * (i+1), x.y * 10* (i+1), x.z));
    s *= s;
  }
  return res;  
}

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
};

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;
  float4 col = Texture0.Sample(LinearSampler, uv);
  float4 blur = Texture1.Sample(LinearSampler, uv);
  float4 credits = Texture2.Sample(LinearSampler, uv);

  float exposure = tonemap.x;
  float minWhite = tonemap.y;

  float creditsLum = Luminance(credits.xyz);

  col = col + smoothstep(0.0, 1, 3 * blur);
  if (creditsLum > 0.1)
    col += credits;

  col = ToneMapReinhard(col, exposure, minWhite);

  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  return col;
}
