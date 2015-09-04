Texture2D texture0 : register(t0);
RWTexture2D<float4> output : register(u0);

cbuffer settings : register(b0)
{
  float2 inputSize : packoffset(c0.x);
  float radius : packoffset(c0.z);
};

[numthreads(256,1,1)]
// entry-point: cs
void BoxBlurY(uint3 globalThreadID : SV_DispatchThreadID)
{
  int x = globalThreadID.x;
  if (x >= inputSize.x)
    return;

  // scale x to -1 .. 1
  float xx = -1 + 2 * ((float)x / inputSize.x);
  float rr = max(0, radius * (0.5 + 0.3 * cos(3.1415 * xx)));

  float scale = 1.0f / (2*rr+1);
  int m = (int)rr;      // integer part of radius
  float alpha = rr - m;   // fractional part

  // load first pixel
  float4 sum = texture0.Load(int3(x, 0, 0));
  int i;
  for (i = 1; i <= m; ++i)
    sum += texture0.Load(int3(x, -i, 0)) + texture0.Load(int3(x, i, 0));
  sum += alpha * (texture0.Load(int3(x, -m-1, 0)) + texture0.Load(int3(x, m+1, 0)));
  
  for (i = 0; i < inputSize.y; ++i)
  {
    output[int2(x, i)] = sum * scale;
    //output[int2(x, i)] = rr;
    sum += lerp(texture0.Load(int3(x, i+m+1, 0)), texture0.Load(int3(x, i+m+2, 0)), alpha);
    sum -= lerp(texture0.Load(int3(x, i-m, 0)), texture0.Load(int3(x, i-m-1, 0)), alpha);
  }
}
