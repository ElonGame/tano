#include "common.hlsl"

cbuffer P : register(b0)
{
  float2 tonemap; // z = exposure/lumAvg, w = min_white
};


float4 Sobel(Texture2D tex, float2 uv)
{
  float4 lum = float4(0.30, 0.59, 0.11, 1);
 
  // TOP ROW
  float s11 = dot(tex.Sample(LinearSampler, uv + float2(-1.0f / 1024.0f, -1.0f / 768.0f)), lum);   // LEFT
  float s12 = dot(tex.Sample(LinearSampler, uv + float2(0, -1.0f / 768.0f)), lum);             // MIDDLE
  float s13 = dot(tex.Sample(LinearSampler, uv + float2(1.0f / 1024.0f, -1.0f / 768.0f)), lum);    // RIGHT
 
  // MIDDLE ROW
  float s21 = dot(tex.Sample(LinearSampler, uv + float2(-1.0f / 1024.0f, 0)), lum);                // LEFT
  // Omit center
  float s23 = dot(tex.Sample(LinearSampler, uv + float2(-1.0f / 1024.0f, 0)), lum);                // RIGHT
 
  // LAST ROW
  float s31 = dot(tex.Sample(LinearSampler, uv + float2(-1.0f / 1024.0f, 1.0f / 768.0f)), lum);    // LEFT
  float s32 = dot(tex.Sample(LinearSampler, uv + float2(0, 1.0f / 768.0f)), lum);              // MIDDLE
  float s33 = dot(tex.Sample(LinearSampler, uv + float2(1.0f / 1024.0f, 1.0f / 768.0f)), lum); // RIGHT
 
  // Filter ... thanks internet <span class="wp-smiley wp-emoji wp-emoji-smile" title=":)">:)</span>
  float t1 = s13 + s33 + (2 * s23) - s11 - (2 * s21) - s31;
  float t2 = s31 + (2 * s32) + s33 - s11 - (2 * s12) - s13;
 
  float4 col;
 
  if (((t1 * t1) + (t2 * t2)) > 0.05) {
  col = float4(0,0,0,1);
  } else {
    col = float4(1,1,1,1);
  }

  return col;
}

// entry-point: ps
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // 0..1
  float2 uv = p.uv.xy;

  // -1..1
  float2 xx = -1 + 2 * uv;

  float4 col = Texture0.Sample(LinearSampler, uv);
  float4 greets = Texture1.Sample(LinearSampler, uv);

  float4 greetsOutline = Sobel(Texture1, uv);

  float greetsLum = Luminance(greets);
  if (greetsLum > 0)
  {
    col = greets * greetsOutline;
    // col = (greets * (1 - 0.5 * col)) - greetsOutline;
  }

  //col *= (greets - greetsOutline);

  //col = greets;

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
