#include "common.hlsl"

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
};

// crazy-trippy sampling :)
float4 sampling(VSQuadOut p)
{
  float2 uv = p.uv.xy;
  // -1..1
  float2 xx = -1 + 2 * uv;
  float2 dd = normalize(float2(0.5, 0.5) - uv);

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);
  float blocker = Texture1.Sample(PointSampler, uv).r;
  float4 orgCol = Texture2.Sample(PointSampler, uv);
  float opacity = Texture3.Sample(PointSampler, uv).x;

  float4 glow = float4(0,0,0,0);
  float2 uvStep = uv;
  float scale = 0.7;
  for (int i = 0; i < 10; ++i)
  {
      float4 backgroundCol = Texture0.Sample(PointSampler, uvStep);
      float blocker = Texture2.Sample(PointSampler, uvStep).r;

      uvStep += 0.1 * dd;
      glow += scale * (1-blocker) * backgroundCol;
      scale *= scale;
  }

  return glow;
}

float4 Volumetric(float2 texCoord, float2 ScreenLightPos)
{  
  // Calculate vector from pixel to light source in screen space.
   float2 deltaTexCoord = (texCoord - ScreenLightPos.xy);

   int NUM_SAMPLES = 20;
   float Density = 1;
   float Weight = 0.5;
   float Decay = 0.5;
   float Exposure = 0.2;

  // Set up illumination decay factor.  
   float illuminationDecay = 0.7;  

  // Divide by number of samples and scale by control factor.  
  deltaTexCoord *= 1.0f / NUM_SAMPLES * Density;  

  // Store initial sample.  
   float3 color = Texture0.Sample(LinearSampler, texCoord).rgb * (1 - Texture1.Sample(LinearSampler, texCoord).r);

  // Evaluate summation from Equation 3 NUM_SAMPLES iterations.  
   for (int i = 0; i < NUM_SAMPLES; i++)  
  {  
    // Step sample location along ray.  
    texCoord -= deltaTexCoord;  
    // Retrieve sample at new location.  
   float3 sample = Texture0.Sample(LinearSampler, texCoord).rgb * (1 - Texture1.Sample(LinearSampler, texCoord).r);
    // Apply sample attenuation scale/decay factors.  
    sample *= illuminationDecay * Weight;  
    // Accumulate combined color.  
    color += sample;  
    // Update exponential decay factor.  
    illuminationDecay *= Decay;  
  }

  return float4(Exposure * color, 1);
}  

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // 0..1
  float2 uv = p.uv.xy;
  // -1..1
  float2 xx = -1 + 2 * uv;
  float2 dd = normalize(float2(0.5, 0.5) - uv);

  float4 backgroundCol = Texture0.Sample(LinearSampler, uv);
  float blocker = Texture1.Sample(LinearSampler, uv).r;
  float4 orgCol = Texture2.Sample(LinearSampler, uv);
  float opacity = Texture3.Sample(LinearSampler, uv).x;

  float4 glow = Volumetric(uv, float2(0.5, 0.5));
  //return glow;

  float4 col = 
    backgroundCol +
    float4(orgCol.rgb / clamp(orgCol.a, 1e-4, 5e4), opacity) + 
    glow;

  float exposure = tonemap.x;
  float minWhite = tonemap.y;
  col = ToneMapReinhard(col, exposure, minWhite);
  
  //  // gamma correction
  col = pow(abs(col), 1.0/2.2);

  // vignette
  float r = 0.8 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  col = r * col;

  return col;
}
