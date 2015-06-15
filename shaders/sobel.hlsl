
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
