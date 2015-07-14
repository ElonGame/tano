#include "common.hlsl"

static float2 halfVec = float2(0.5, 0.5);
static float halfVecLength = length(halfVec);

cbuffer LensFlare : register(b1)
{
  // x = dispersion
  // y = num_ghosts
  float4 lensFlareParams;
};

float4 PsLensFlare(VSQuadOut p) : SV_Target
{
  // texture0 = scale/biased input

  // flip texcoords
  float2 uv = -p.uv.xy + float2(1,1);

  // vector towards center of screen
  float dispersal = lensFlareParams.x;
  int numGhosts = (int)lensFlareParams.y;
  float haloWidth = lensFlareParams.z;
  float strength = lensFlareParams.w;
  float2 dir = dispersal * (halfVec - uv);
  float2 haloVec = normalize(dir) * haloWidth;

  float haloWeight = length(halfVec - frac(uv + haloVec)) / halfVecLength;
  haloWeight = pow(max(0, 1.0 - haloWeight), 5.0);
  float4 halo = Texture0.Sample(LinearWrap, uv + haloVec) * haloWeight;
  float4 res = halo;

  for (int i = 0; i < numGhosts; ++i)
  {
    float2 ofs = uv + (float)i * dir;
    float weight = length(halfVec - ofs) / length(float2(0.5, 0.5));
    weight = pow(max(0, 1.0 - weight), 10.0);
    res += Texture0.Sample(LinearWrap, uv + (float)i * dir);
  }
  return strength * res;
}
