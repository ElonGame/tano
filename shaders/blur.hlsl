// Gaussian blur, based on http://fgiesen.wordpress.com/2012/08/01/fast-blurs-2/

// Baseline: 5130 us

Texture2D texture0 : register(t0);
RWTexture2D<float> scratch1 : register(u0);
RWTexture2D<float> scratch2 : register(u1);
RWTexture2D<float4> output : register(u2);

cbuffer settings : register(b0)
{
  float2 inputSize : packoffset(c0.x);
  float radius : packoffset(c0.z);
};

void WriteFloat4(RWTexture2D<float> tex, int x, int y, float4 val)
{
  tex[int2(x*4+0, y)] = val.x;  
  tex[int2(x*4+1, y)] = val.y;  
  tex[int2(x*4+2, y)] = val.z;  
  tex[int2(x*4+3, y)] = val.w;  
}

float4 ReadFloat4(RWTexture2D<float> tex, int x, int y)
{
  float4 res;
  res.x = tex[int2(x*4+0, y)];
  res.y = tex[int2(x*4+1, y)];
  res.z = tex[int2(x*4+2, y)];
  res.w = tex[int2(x*4+3, y)];
  return res;
}

[numthreads(256,1,1)]
void BoxBlurX(uint3 globalThreadID : SV_DispatchThreadID)
{
  int y = globalThreadID.x;
  if (y >= inputSize.y) 
    return;

  float scale = 1.0f / (2*radius+1);
  int m = (int)radius;        // integer part of radius
  float alpha = radius - m;   // fractional part

  float4 sum;
  int i;

  // Do 3 blur passes. input -> scratch1, scratch1 -> scratch2, scratch2 -> output
  // load first pixel
  sum = texture0.Load(int3(0, y, 0));
  for (i = 1; i <= m; ++i)
    sum += texture0.Load(int3(-i, y, 0)) + texture0.Load(int3(i, y, 0));
  sum += alpha * (texture0.Load(int3(-m-1, y, 0)) + texture0.Load(int3(m+1, y, 0)));
  
  for (i = 0; i < inputSize.x; ++i)
  {
    WriteFloat4(scratch1, i, y, sum * scale);
    sum += lerp(texture0.Load(int3(i+m+1, y, 0)), texture0.Load(int3(i+m+2, y, 0)), alpha);
    sum -= lerp(texture0.Load(int3(i-m, y, 0)), texture0.Load(int3(i-m-1, y, 0)), alpha);
  }

  // pass 2
  sum = ReadFloat4(scratch1, 0, y);
  for (i = 1; i <= m; ++i)
  {
    sum += ReadFloat4(scratch1, -i, y) + ReadFloat4(scratch1, +i, y);
  }
  sum += alpha * (ReadFloat4(scratch1, -m-1, y) + ReadFloat4(scratch1, m+1, y));
  
  for (i = 0; i < inputSize.x; ++i)
  {
    WriteFloat4(scratch2, i, y, sum * scale);
    sum += lerp(ReadFloat4(scratch1, i+m+1, y), ReadFloat4(scratch1, i+m+2, y), alpha);
    sum -= lerp(ReadFloat4(scratch1, i-m, y), ReadFloat4(scratch1, i-m-1, y), alpha);
  }

  // pass 3
  sum = ReadFloat4(scratch2, 0, y);

  for (i = 1; i <= m; ++i)
  {
    sum += ReadFloat4(scratch2, -i, y) + ReadFloat4(scratch2, +i, y);
  }
  sum += alpha * (ReadFloat4(scratch2, -m-1, y) + ReadFloat4(scratch2, m+1, y));
  
  for (i = 0; i < inputSize.x; ++i)
  {
    // note, final output is transposed
    output[int2(y, i)] = sum * scale;

    sum += lerp(ReadFloat4(scratch2, i+m+1, y), ReadFloat4(scratch2, i+m+2, y), alpha);
    sum -= lerp(ReadFloat4(scratch2, i-m, y), ReadFloat4(scratch2, i-m-1, y), alpha);
  }
}
