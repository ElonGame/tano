// Gaussian blur, based on http://fgiesen.wordpress.com/2012/08/01/fast-blurs-2/

// Baseline: 5130 us
// Using BLurTranspose: 3800 us

Texture2D texture0 : register(t0);
RWTexture2D<float4> output : register(u0);

cbuffer settings : register(b0)
{
  float2 inputSize : packoffset(c0.x);
  float radius : packoffset(c0.z);
};

[numthreads(256,1,1)]
// entry-point: cs
void CopyTranspose(uint3 globalThreadID : SV_DispatchThreadID)
{
  int y = globalThreadID.x;
  if (y >= inputSize.y) 
    return;
    
  int i;
  for (i = 0; i < inputSize.x; ++i)
  {
    float4 color = texture0.Load(int3(i, y, 0));
    output[int2(y,i)] = color;
  }
}

[numthreads(256,1,1)]
// entry-point: cs
void BlurTranspose(uint3 globalThreadID : SV_DispatchThreadID)
{
  int y = globalThreadID.x;
  if (y >= inputSize.y) 
    return;

  float scale = 1.0f / (2*radius+1);
  int m = (int)radius;      // integer part of radius
  float alpha = radius - m;   // fractional part

  // load first pixel
  float4 sum = texture0.Load(int3(0, y, 0));
  int i;
  for (i = 1; i <= m; ++i)
    sum += texture0.Load(int3(-i, y, 0)) + texture0.Load(int3(i, y, 0));
  sum += alpha * (texture0.Load(int3(-m-1, y, 0)) + texture0.Load(int3(m+1, y, 0)));
  
  for (i = 0; i < inputSize.x; ++i)
  {
    // output the pixel
    output[int2(y, i)] = sum * scale;
    sum += lerp(texture0.Load(int3(i+m+1, y, 0)), texture0.Load(int3(i+m+2, y, 0)), alpha);
    sum -= lerp(texture0.Load(int3(i-m, y, 0)), texture0.Load(int3(i-m-1, y, 0)), alpha);
  }
}

[numthreads(256,1,1)]
// entry-point: cs
void BoxBlurX(uint3 globalThreadID : SV_DispatchThreadID)
{
  int y = globalThreadID.x;
  if (y >= inputSize.y) 
    return;

  float rr = radius;
  //float rr = saturate(radius * sin((float)y / 10));
  //rr = y / 50;

  float scale = 1.0f / (2*rr+1);
  int m = (int)rr;      // integer part of radius
  float alpha = rr - m;   // fractional part

  // load first pixel
  float4 sum = texture0.Load(int3(0, y, 0));
  int i;
  for (i = 1; i <= m; ++i)
    sum += texture0.Load(int3(-i, y, 0)) + texture0.Load(int3(i, y, 0));
  sum += alpha * (texture0.Load(int3(-m-1, y, 0)) + texture0.Load(int3(m+1, y, 0)));
  
  for (i = 0; i < inputSize.x; ++i)
  {
    output[int2(i,y)] = sum * scale;
    sum += lerp(texture0.Load(int3(i+m+1, y, 0)), texture0.Load(int3(i+m+2, y, 0)), alpha);
    sum -= lerp(texture0.Load(int3(i-m, y, 0)), texture0.Load(int3(i-m-1, y, 0)), alpha);
  }
}

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
