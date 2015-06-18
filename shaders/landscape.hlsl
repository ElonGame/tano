#include "common.hlsl"

cbuffer PerFrame : register(b0)
{
  matrix world;
  matrix view;
  matrix proj;
  matrix viewProj;
  float4 time;
  float4 dim;
  float3 cameraPos;
  float3 cameraLookAt;
  float3 cameraUp;
  float4 nearFar : NEAR_FAR;
  float4 tonemap; // x = shoulder, y = max_white
};

struct PsColBrightnessOut
{
  float4 col : SV_Target0;
  // rgb = bloom
  // a = depth
  float4 extra : SV_Target1;
};


static float4 BOID_COLOR = float4(0.4, 0.2, 0.2, 1);
static float3 FOG_COLOR = 0.5 * float3(0.5, 0.6, 0.7);
static float3 SUN_COLOR = 0.5 * float3(1.5, 0.9, 0.3);
static float3 SUN_DIR = normalize(float3(0, 0, -1));
static float3 SUN_POS = float3(0, 0, 2000);

//------------------------------------------------------
// sky
//------------------------------------------------------

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, 30));
  return fogColor;
}

PsColBrightnessOut PsSky(VSQuadOut p)
{
  float2 uv = p.uv.xy;
  float2 r = -1 + 2 * uv;

  float f = 1;

  // calc camera base
  float3 dir = normalize(cameraLookAt - cameraPos);
  float3 up = float3(0,1,0);
  float3 right = cross(up, dir);
  up = cross(right, dir);

  // calc ray dir 
  // this is done by determining which pixel on the image
  // plane the current pixel represents
  float aspectRatio = dim.x / dim.y;
  float3 rayDir = 2 * (-0.5 + p.pos.x / dim.x) * right + (2 * (-0.5 + p.pos.y / dim.y)) * up + f * dir;
  rayDir.y /= aspectRatio;
  rayDir = normalize(rayDir);

  PsColBrightnessOut res;
  float3 tmp = FogColor(rayDir);
  res.col = float4(tmp, 1);
  res.extra.rgb = pow(max(0, Luminance(tmp)), 5);
  res.extra.a = p.pos.z;
  return res;
}

//------------------------------------------------------
// particles
//------------------------------------------------------

struct VsParticleIn
{
  float3 pos : Position;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord0;
  float z : TexCoord1;
  float alpha : TexCoord2;
};

// 1--2
// |  |
// 0--3

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 1), float2(1, 0)
};

VsParticleIn VsParticle(VsParticleIn v)
{
  VsParticleIn res;
  res.pos = v.pos;
  return res;
}

[maxvertexcount(4)]
void GsParticle(point VsParticleIn input[1], inout TriangleStream<VsParticleOut> outStream)
{
  // Note, the DirectX strip order differs from my usual order. It might be
  // a good idea to change my stuff..
  // 1--3
  // |  |
  // 0--2

  static float s = 0.75;
  static float3 ofs0 = float3(-s, -s, 0);
  static float3 ofs1 = float3(-s, +s, 0);
  static float3 ofs2 = float3(+s, +s, 0);
  static float3 ofs3 = float3(+s, -s, 0);

  matrix worldViewProj = mul(world, viewProj);

  float3 pos = input[0].pos;
  float3 dir = normalize(cameraPos - pos);
  float3 right = cross(dir, float3(0,1,0));
  float3 up = cross(right, dir);

  VsParticleOut p;
  p.alpha = 1;
  float3 p0 = float3(pos - s * right - s * up);
  float3 p1 = float3(pos - s * right + s * up);
  float3 p2 = float3(pos + s * right - s * up);
  float3 p3 = float3(pos + s * right + s * up);
  p.pos = mul(float4(p0, 1), worldViewProj);
  p.uv = uvsVtx[0];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p1, 1), worldViewProj);
  p.uv = uvsVtx[1];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p2, 1), worldViewProj);
  p.uv = uvsVtx[2];
  p.z = p.pos.z;
  outStream.Append(p);

  p.pos = mul(float4(p3, 1), worldViewProj);
  p.uv = uvsVtx[3];
  p.z = p.pos.z;
  outStream.Append(p);
}

static float SoftParticleContrast = 2.0;
static float intensity = 1.0;
static float zEpsilon = 0.0;

PsColBrightnessOut PsParticle(VsParticleOut p)
{
  // Texture0 = particle texture
  // Texture1 = zbuffer
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  col.g *= 0.6;
  col.b *= 0.1;
  float zBuf = Texture1.Load(int3(p.pos.x, p.pos.y, 0)).r;

  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float farClip = nearFar.y;
  float f_mul_n = nearFar.z;
  float f_sub_n = nearFar.w;
  float z = f_mul_n / ( farClip - zBuf * f_sub_n);
  float zdiff = (z - p.z);
  float c = smoothstep(0, 1, zdiff / SoftParticleContrast);
  if( c * zdiff <= zEpsilon )
      discard;

  PsColBrightnessOut res;
  res.col = intensity * c * col;
  res.extra.rgb = intensity * c * col.rgb;
  res.extra.a = p.z;
  return res;
}

//------------------------------------------------------
// landscape
//------------------------------------------------------

struct VsLandscapeIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsLandscapeOut
{
    float4 pos  : SV_Position;
    float3 normal : Normal;
    float3 rayDir : Texture0;
    float distance : TexCoord1;
};

struct PsLandscapeIn
{
    float4 pos : SV_POSITION;
    noperspective float4 EdgeA: TEXCOORD1;
    noperspective float4 EdgeB: TEXCOORD2;
    uint Case : TEXCOORD3;
    float3 normal : NORMAL;
    float3 rayDir : TEXCOORD4;
    float distance : TEXCOORD5;
};

VsLandscapeOut VsLandscape(VsLandscapeIn v)
{
  VsLandscapeOut res;
  matrix worldViewProj = mul(world, viewProj);

  // This is the biggest cheese, but if I don't do it, everything goes to hell..
  float3 pos = v.pos;
  if (pos.z == cameraPos.z)
    pos.z += 0.01;

  res.pos = mul(float4(pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  float3 dir = pos - cameraPos;
  res.distance = length(dir);
  res.rayDir = dir * 1/res.distance;
  return res;
}

static float4 LightVector = float4( 0, 0, 1, 0);
static float4 FillColor = float4(0.1, 0.2, 0.4, 1);
static float4 WireColor = float4(1, 1, 1, 1);
static float LineWidth = 2.5;

float2 projToWindow(in float4 pos)
{
    return float2(  dim.x*0.5*((pos.x/pos.w) + 1) + dim.z,
                    dim.y*0.5*(1-(pos.y/pos.w)) + dim.w );
}


[maxvertexcount(3)]
void GsLandscape(triangle VsLandscapeOut input[3], inout TriangleStream<PsLandscapeIn> outStream)
{
    PsLandscapeIn output;

    // Compute the case from the positions of point in space.
    output.Case = (input[0].pos.z <= 0)*4 + (input[1].pos.z <= 0)*2 + (input[2].pos.z <= 0);

    // If case is all vertices behind viewpoint (case = 7) then cull.
    if (output.Case == 7)
      return;

   // Transform position to window space
    float2 points[3];
    points[0] = projToWindow(input[0].pos);
    points[1] = projToWindow(input[1].pos);
    points[2] = projToWindow(input[2].pos);

    // If Case is 0, all projected points are defined, do the
    // general case computation
    if (output.Case == 0) 
    {
        output.EdgeA = float4(0,0,0,0);
        output.EdgeB = float4(0,0,0,0);

        // Compute the edges vectors of the transformed triangle
        float2 edges[3];
        edges[0] = points[1] - points[0];
        edges[1] = points[2] - points[1];
        edges[2] = points[0] - points[2];

        // Store the length of the edges
        float lengths[3];
        lengths[0] = length(edges[0]);
        lengths[1] = length(edges[1]);
        lengths[2] = length(edges[2]);

        // For each vertex, compute the angle between the two edges
        // a.b = |a||b|cos(theta)
        // Note, the edges are directed, which is why one edge is negated
        float cosAngles[3];
        cosAngles[0] = dot( -edges[2], edges[0]) / ( lengths[2] * lengths[0] );
        cosAngles[1] = dot( -edges[0], edges[1]) / ( lengths[0] * lengths[1] );
        cosAngles[2] = dot( -edges[1], edges[2]) / ( lengths[1] * lengths[2] );

        // To compute the height for each vertex, use ex:
        // sin(P2) = h1/e2 => h1 = e2 * sin(P2)
        // and: sin(theta) = sqrt(1 - cos^2(theta))
        output.pos =( input[0].pos );
        output.EdgeA[0] = 0;
        output.EdgeA[1] = lengths[2]*sqrt(1 - cosAngles[2]*cosAngles[2]);
        output.EdgeA[2] = 0;
        output.normal = input[0].normal;
        output.rayDir = input[0].rayDir;
        output.distance = input[0].distance;
        outStream.Append( output );

        output.pos = ( input[1].pos );
        output.EdgeA[0] = 0;
        output.EdgeA[1] = 0;
        output.EdgeA[2]= lengths[0]*sqrt(1 - cosAngles[0]*cosAngles[0]);
        output.normal = input[1].normal;
        output.rayDir = input[1].rayDir;
        output.distance = input[1].distance;
        outStream.Append( output );

        output.pos = ( input[2].pos );
        output.EdgeA.x = lengths[1]*sqrt(1 - cosAngles[1]*cosAngles[1]);
        output.EdgeA[1] = 0;
        output.EdgeA[2] = 0;
        output.normal = input[2].normal;
        output.rayDir = input[2].rayDir;
        output.distance = input[2].distance;
        outStream.Append( output );

        outStream.RestartStrip();
    }
    // Else need some tricky computations
    else
    {
        //                   0  1  2  3  4  5  6
        uint infoA[]     = { 0, 0, 0, 0, 1, 1, 2 };
        uint infoB[]     = { 1, 1, 2, 0, 2, 1, 2 };
        uint infoAd[]    = { 2, 2, 1, 1, 0, 0, 0 };
        uint infoBd[]    = { 2, 2, 1, 2, 0, 2, 1 }; 

        // Get the visible vertices
        output.EdgeA.xy = points[ infoA[output.Case] ];
        output.EdgeB.xy = points[ infoB[output.Case] ];

        // Compute edges to non-visible vertices
        output.EdgeA.zw = normalize( output.EdgeA.xy - points[ infoAd[output.Case] ] ); 
        output.EdgeB.zw = normalize( output.EdgeB.xy - points[ infoBd[output.Case] ] );
    
        // Generate vertices
        output.pos =( input[0].pos );
        output.normal = input[0].normal;
        output.rayDir = input[0].rayDir;
        output.distance = input[0].distance;
        outStream.Append( output );
     
        output.pos = ( input[1].pos );
        output.normal = input[1].normal;
        output.rayDir = input[1].rayDir;
        output.distance = input[1].distance;
        outStream.Append( output );

        output.pos = ( input[2].pos );
        output.normal = input[2].normal;
        output.rayDir = input[2].rayDir;
        output.distance = input[2].distance;
        outStream.Append( output );

        outStream.RestartStrip();
    }
}

float MainDistanceToEdge(in PsLandscapeIn input)
{
    float dist;

    // The easy case, the 3 distances of the fragment to the 3 edges is already
    // computed, get the min.
    if (input.Case == 0)
    {
        dist = min ( min (input.EdgeA.x, input.EdgeA.y), input.EdgeA.z);
    }
    // The tricky case, compute the distances and get the min from the 2D lines
    // given from the geometry shader.
    else
    {
        // Compute and compare the sqDist, do one sqrt in the end.
          
        float2 AF = input.pos.xy - input.EdgeA.xy;
        float sqAF = dot(AF,AF);
        float AFcosA = dot(AF, input.EdgeA.zw);
        dist = abs(sqAF - AFcosA*AFcosA);

        float2 BF = input.pos.xy - input.EdgeB.xy;
        float sqBF = dot(BF,BF);
        float BFcosB = dot(BF, input.EdgeB.zw);
        dist = min( dist, abs(sqBF - BFcosB*BFcosB) );
       
        // Only need to care about the 3rd edge for some cases.
        if (input.Case == 1 || input.Case == 2 || input.Case == 4)
        {
            float AFcosA0 = dot(AF, normalize(input.EdgeB.xy - input.EdgeA.xy));
            dist = min( dist, abs(sqAF - AFcosA0*AFcosA0) );
        }

        dist = sqrt(dist);
    }

    return dist;
}

static float4 ColorCases[] = {
    { 1, 1, 1, 1 }, 
    { 1, 1, 0, 1 },
    { 1, 0, 1, 1 },
    { 1, 0, 0, 1 },
    { 0, 1, 1, 1 },
    { 0, 1, 0, 1 },
    { 0, 0, 1, 1 }
}; 


PsColBrightnessOut PsLandscape(PsLandscapeIn p)
{
    // Compute the shortest distance between the fragment and the edges.
    float dist = MainDistanceToEdge(p);
    //return ColorCases[p.Case];

    // Map the computed distance to the [0,2] range on the border of the line.
    dist = clamp((dist - (0.5*LineWidth - 1)), 0, 2);

    // Alpha is computed from the function exp2(-2(x)^2).
    dist *= dist;
    float alpha = exp2(-2*dist);


    float3 amb = float3(0.05, 0.05, 0.05);
    float dff = saturate(dot(-SUN_DIR, p.normal));
    float3 col = amb + dff * float3(0.1, 0.1, 0.25);

    col = lerp(col, WireColor.rgb, alpha);

    float b = 0.002;
    float fogAmount = max(0, 1 - exp(-(p.distance) * b));
    float3 fogColor = FogColor(p.rayDir);

    PsColBrightnessOut res;
    col = lerp(col, fogColor, fogAmount);
    res.col = float4(col, 0.9);
    float lum = Luminance(col);
    res.extra.rgb = smoothstep(0.7, 1, lum);
    res.extra.a = p.pos.z;
    return res;
}

//------------------------------------------------------
// boids
//------------------------------------------------------
struct VsBoidsIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsBoidsOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
};

VsBoidsOut VsBoids(VsBoidsIn p)
{
  VsBoidsOut output;
  float4x4 mtxWorldViewProj = mul(world, viewProj);
  output.pos = mul(float4(p.pos, 1), mtxWorldViewProj);
  output.normal = mul(float4(p.normal, 0), world).xyz;
  output.normal = p.normal;
  return output;
}

float4 PsBoids(VsBoidsOut p) : SV_Target
{
  return BOID_COLOR * max(0,(dot(normalize(p.normal), float3(0,-1,0))));
}

//------------------------------------------------------
// high pass filter
//------------------------------------------------------
float4 PsHighPassFilter(VSQuadOut p) : SV_Target
{
  // Texture0 = color
  // Texture1 = brightness
  float4 col = Texture0.Sample(PointSampler, p.uv);
  float4 brightness = Texture1.Sample(PointSampler, p.uv);
  return Luminance(col.xyz) > 0.4 ? pow(col, 0.5) : 0;
}

float4 ToneMap(float4 col)
{
  float lum = Luminance(col.rgb);
  float shoulder = tonemap.x;
  float maxWhite = tonemap.y;
  return col * lum / ((1-shoulder) * maxWhite + shoulder * lum);
}

//------------------------------------------------------
// composite
//------------------------------------------------------
float4 PsComposite(VSQuadOut p) : SV_Target
{
  // Texture0 : color + bloom
  // Texture1 : color + bloom blurred
  // Texture2 : zbuffer
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

  float4 col = Texture0.Sample(PointSampler, uv) + Texture1.Sample(PointSampler, uv);
/*
  // convert sun to screen space
  float4 sunProjSpace = mul(float4(SUN_POS, 1), viewProj);
  float2 sunScreenSpace = projToWindow(sunProjSpace);

  float2 sunDir = p.pos.xy - sunScreenSpace;
  int NUM_STEPS = 40;
  float2 sunStep = sunDir / NUM_STEPS;

  //float4 res = Texture0.Sample(PointSampler, uv);
  int i;
  float2 curPos = p.pos.xy;
  float curDecay = 1.0;
  float decay = 0.5;
  float weight = 0.5;
  float4 res = 0;
  for (i = 0; i < NUM_STEPS; ++i)
  {
    curPos -= sunStep;
    float4 s = Texture0.Sample(PointSampler, curPos) * weight * curDecay;
    res += s;
    curDecay *= decay;
  }
  //return res;
*/
  /*
  float4 dof = Texture1.Sample(PointSampler, uv);
  float zBuf = Texture2.Load(int3(p.pos.x, p.pos.y, 0)).r;

  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float farClip = nearFar.y;
  float f_mul_n = nearFar.z;
  float f_sub_n = nearFar.w;
  float z = f_mul_n / ( farClip - zBuf * f_sub_n);
//  z /= farClip;

  float nearStart = 0;
  float nearEnd = 200;
  float farStart = 1000;
  float farEnd = 2000;
  float t = min(smoothstep(nearStart, nearEnd, z), 1 - smoothstep(farStart, farEnd, z));

  //col = lerp(col, dof, t);
//  return t;
//  return z;
*/

  col = ToneMap(col);
  
   // gamma correction
  col = pow(abs(col), 1.0/2.2);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * col;
}
