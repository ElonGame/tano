Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
Texture2D Texture3 : register(t3);

// Samplers if using the GpuObjects samplers
sampler PointSampler : register(s0);
sampler LinearSampler : register(s1);
sampler LinearWrap : register(s2);
sampler LinearBorder : register(s3);


//------------------------------------------------------
float Luminance(float3 col)
{
  return 0.2126 * col.r + 0.7152 * col.g + 0.0722 * col.b;
}

//------------------------------------------------------
float4 ToneMapReinhard(float4 col, float exposure, float minWhite)
{
  float lum = Luminance(col.rgb);

  // scaled to middle gray (but using a hard coded value)
  float ll = lum * exposure; // this is exposure / lumAvg
  float w = minWhite * minWhite;

  float lumTonemapped = ll * (1.0 + (ll / (w * w))) / (1.0 + ll);

  float scale = lumTonemapped / lum;
  return saturate(scale * col);
}

//------------------------------------------------------
struct VSQuadOut
{
  float4 pos : SV_Position;
  float2 uv: TexCoord;
};

//------------------------------------------------------
// outputs a full screen triangle with screen-space coordinates
// input: three empty vertices
VSQuadOut VsQuad(uint vertexID : SV_VertexID)
{
    VSQuadOut result;
  // ID=0 -> Pos=[-1,-1], Tex=[0,0]
  // ID=1 -> Pos=[ 3,-1], Tex=[2,0]
  // ID=2 -> Pos=[-1,-3], Tex=[0,2]
    result.uv = float2((vertexID << 1) & 2, vertexID & 2);
    result.pos = float4(result.uv * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
    return result;
}

//------------------------------------------------------
float4 PsCopy(VSQuadOut p) : SV_Target
{
  return Texture0.Sample(LinearSampler, p.uv);
}

//------------------------------------------------------
float4 PsAdd(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 a = Texture0.Sample(PointSampler, uv);
  float4 b = Texture1.Sample(PointSampler, uv);
  return a + b;
}

