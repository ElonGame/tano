#include "common.hlsl"

Texture2D Texture0 : register(t0);
Texture2D Texture1 : register(t1);
Texture2D Texture2 : register(t2);
sampler PointSampler : register(s0);

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
};

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

struct VsLandscapeIn
{
  float3 pos : Position;
  float3 normal : Normal;
};

struct VsLandscapeOut
{
  float4 pos : SV_Position;
  float3 normal : Normal;
  float3 rayDir : Texture0;
  float distance : TexCoord1;
  float distance2 : Texture2;
};

struct VsParticleIn
{
  float3 pos : Position;
};

struct VsParticleOut
{
  float4 pos : SV_Position;
  float2 uv : TexCoord;
};


static float4 BOID_COLOR = float4(0.4, 0.2, 0.2, 1);
static float3 FOG_COLOR = 0.5 * float3(0.5, 0.6, 0.7);
static float3 SUN_COLOR = 0.5 * float3(1.5, 0.9, 0.3);
static float3 SUN_DIR = normalize(float3(-0.5, -0.2, -0.4));
//static float3 SUN_DIR = normalize(float3(0, -0.2, -1));

//------------------------------------------------------
// sky
//------------------------------------------------------

float3 FogColor(float3 rayDir)
{
  float sunAmount = max(0, dot(rayDir, -SUN_DIR));
  float3 fogColor = lerp(FOG_COLOR, SUN_COLOR, pow(sunAmount, 20));
  return fogColor;
}

float4 PsSky(VSQuadOut p) : SV_Target
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

  return float4(FogColor(rayDir), 1);
}

//------------------------------------------------------
// particles
//------------------------------------------------------

// 1--2
// |  |
// 0--3

static float2 uvsVtx[4] = {
  float2(0, 1), float2(0, 0), float2(1, 0), float2(1, 1)
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

  // 1--2
  // |  |
  // 0--3

  static float s = 1;
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
  p.pos = mul(float4(pos - s * right - s * up, 1), worldViewProj);
  p.uv = uvsVtx[0];
  outStream.Append(p);

  p.pos = mul(float4(pos - s * right + s * up, 1), worldViewProj);
  p.uv = uvsVtx[1];
  outStream.Append(p);

  p.pos = mul(float4(pos + s * right + s * up, 1), worldViewProj);
  p.uv = uvsVtx[2];
  outStream.Append(p);

  p.pos = mul(float4(pos + s * right - s * up, 1), worldViewProj);
  p.uv = uvsVtx[3];
  outStream.Append(p);

  outStream.RestartStrip();
}

float4 PsParticle(VsParticleOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float4 col = Texture0.Sample(PointSampler, uv);
  float f = dot(col.xyz, float3(1.0/3, 1.0/3, 1.0/3));

  return float4(f * float3(1, 1, 0), col.a);
}


//------------------------------------------------------
// landscape
//------------------------------------------------------

VsLandscapeOut VsLandscape(VsLandscapeIn v)
{
  VsLandscapeOut res;
  matrix worldViewProj = mul(world, viewProj);

  float3 pos = v.pos;
  float3 center = float3(time.y, time.z, time.w);
  float r = length(center - v.pos) / 10;

  res.pos = mul(float4(pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;

  float3 dir = pos - cameraPos;
  res.distance = length(dir);
  res.rayDir = dir * 1/res.distance;
  res.distance2 = r;
  return res;
}

struct GS_INPUT
{
    float4 Pos  : POSITION;
    float3 normal : Normal;
    float3 rayDir : Texture0;
    float distance : TexCoord1;
};

GS_INPUT VsLandscape2(VsLandscapeIn v)
{
  GS_INPUT res;
  matrix worldViewProj = mul(world, viewProj);

  res.Pos = mul(float4(v.pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  float3 dir = v.pos - cameraPos;
  res.distance = length(dir);
  res.rayDir = dir * 1/res.distance;
  return res;
}

float4 PsLandscape(VsLandscapeOut p) : SV_Target
{
  float3 amb = float3(0.05, 0.05, 0.05);
  float dff = saturate(dot(-SUN_DIR, p.normal));
//  float3 col = amb + dff * float3(0.1, 0.1, 0.25);
  float3 col = amb + dff * (1-saturate(p.distance2 / 10)) * float3(0.1, 0.1, 0.25);

  float b = 0.001;
  float fogAmount = max(0, 1 - exp(-(p.distance - 500) * b));

  float3 fogColor = FogColor(p.rayDir);

  return float4(lerp(col, fogColor, fogAmount), 1);
}

//--------------------------------------------------------------------------------------
// Geometry Shader
//--------------------------------------------------------------------------------------

struct PS_INPUT_WIRE
{
    float4 Pos : SV_POSITION;
//    float4 Col : TEXCOORD0;
    noperspective float4 EdgeA: TEXCOORD1;
    noperspective float4 EdgeB: TEXCOORD2;
    uint Case : TEXCOORD3;
    float3 normal : NORMAL;
    float3 rayDir : TEXCOORD4;
    float distance : TEXCOORD5;
};


static float4 LightVector = float4( 0, 0, 1, 0);
static float4 FillColor = float4(0.1, 0.2, 0.4, 1);
static float4 WireColor = float4(1, 1, 1, 1);
static float LineWidth = 1.5;

static uint infoA[]     = { 0, 0, 0, 0, 1, 1, 2 };
static uint infoB[]     = { 1, 1, 2, 0, 2, 1, 2 };
static uint infoAd[]    = { 2, 2, 1, 1, 0, 0, 0 };
static uint infoBd[]    = { 2, 2, 1, 2, 0, 2, 1 }; 
static uint infoEdge0[] = { 0, 2, 0, 0, 0, 0, 2 }; 

float2 projToWindow(in float4 pos)
{
    return float2(  dim.x*0.5*((pos.x/pos.w) + 1) + dim.z,
                    dim.y*0.5*(1-(pos.y/pos.w)) + dim.w );
}

[maxvertexcount(3)]
void GsSolidWire( triangle GS_INPUT input[3],
                         inout TriangleStream<PS_INPUT_WIRE> outStream )
{
    PS_INPUT_WIRE output;

    // Compute the case from the positions of point in space.
    output.Case = (input[0].Pos.z < 0)*4 + (input[1].Pos.z < 0)*2 + (input[2].Pos.z < 0); 

    // If case is all vertices behind viewpoint (case = 7) then cull.
    if (output.Case == 7) return;

   // Transform position to window space
    float2 points[3];
    points[0] = projToWindow(input[0].Pos);
    points[1] = projToWindow(input[1].Pos);
    points[2] = projToWindow(input[2].Pos);

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

        // Compute the cos angle of each vertices
        float cosAngles[3];
        cosAngles[0] = dot( -edges[2], edges[0]) / ( lengths[2] * lengths[0] );
        cosAngles[1] = dot( -edges[0], edges[1]) / ( lengths[0] * lengths[1] );
        cosAngles[2] = dot( -edges[1], edges[2]) / ( lengths[1] * lengths[2] );

        // The height for each vertices of the triangle
        float heights[3];
        heights[1] = lengths[0]*sqrt(1 - cosAngles[0]*cosAngles[0]);
        heights[2] = lengths[1]*sqrt(1 - cosAngles[1]*cosAngles[1]);
        heights[0] = lengths[2]*sqrt(1 - cosAngles[2]*cosAngles[2]);

        float edgeSigns[3];
        edgeSigns[0] = (edges[0].x > 0 ? 1 : -1);
        edgeSigns[1] = (edges[1].x > 0 ? 1 : -1);
        edgeSigns[2] = (edges[2].x > 0 ? 1 : -1);

        float edgeOffsets[3];
        edgeOffsets[0] = lengths[0]*(0.5 - 0.5*edgeSigns[0]);
        edgeOffsets[1] = lengths[1]*(0.5 - 0.5*edgeSigns[1]);
        edgeOffsets[2] = lengths[2]*(0.5 - 0.5*edgeSigns[2]);

        output.Pos =( input[0].Pos );
        output.EdgeA[0] = 0;
        output.EdgeA[1] = heights[0];
        output.EdgeA[2] = 0;
        output.EdgeB[0] = edgeOffsets[0];
        output.EdgeB[1] = edgeOffsets[1] + edgeSigns[1] * cosAngles[1]*lengths[0];
        output.EdgeB[2] = edgeOffsets[2] + edgeSigns[2] * lengths[2];
        output.normal = input[0].normal;
        output.rayDir = input[0].rayDir;
        output.distance = input[0].distance;
        outStream.Append( output );

        output.Pos = ( input[1].Pos );
        output.EdgeA[0] = 0;
        output.EdgeA[1] = 0;
        output.EdgeA[2] = heights[1];
        output.EdgeB[0] = edgeOffsets[0] + edgeSigns[0] * lengths[0];
        output.EdgeB[1] = edgeOffsets[1];
        output.EdgeB[2] = edgeOffsets[2] + edgeSigns[2] * cosAngles[2]*lengths[1];
        output.normal = input[1].normal;
        output.rayDir = input[1].rayDir;
        output.distance = input[1].distance;
        outStream.Append( output );

        output.Pos = ( input[2].Pos );
        output.EdgeA[0] = heights[2];
        output.EdgeA[1] = 0;
        output.EdgeA[2] = 0;
        output.EdgeB[0] = edgeOffsets[0] + edgeSigns[0] * cosAngles[0]*lengths[2];
        output.EdgeB[1] = edgeOffsets[1] + edgeSigns[1] * lengths[1];
        output.EdgeB[2] = edgeOffsets[2];
        output.normal = input[2].normal;
        output.rayDir = input[2].rayDir;
        output.distance = input[2].distance;
        outStream.Append( output );

        outStream.RestartStrip();
    }
    // Else need some tricky computations
    else
    {
        // Then compute and pass the edge definitions from the case
        output.EdgeA.xy = points[ infoA[output.Case] ];
        output.EdgeB.xy = points[ infoB[output.Case] ];

        output.EdgeA.zw = normalize( output.EdgeA.xy - points[ infoAd[output.Case] ] ); 
        output.EdgeB.zw = normalize( output.EdgeB.xy - points[ infoBd[output.Case] ] );
    
    // Generate vertices
        output.Pos =( input[0].Pos );
        output.normal = input[0].normal;
        output.rayDir = input[0].rayDir;
        output.distance = input[0].distance;
        outStream.Append( output );
     
        output.Pos = ( input[1].Pos );
        output.normal = input[1].normal;
        output.rayDir = input[1].rayDir;
        output.distance = input[1].distance;
        outStream.Append( output );

        output.Pos = ( input[2].Pos );
        output.normal = input[2].normal;
        output.rayDir = input[2].rayDir;
        output.distance = input[2].distance;
        outStream.Append( output );

        outStream.RestartStrip();
    }
}

float evalMinDistanceToEdges(in PS_INPUT_WIRE input)
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
          
        float2 AF = input.Pos.xy - input.EdgeA.xy;
        float sqAF = dot(AF,AF);
        float AFcosA = dot(AF, input.EdgeA.zw);
        dist = abs(sqAF - AFcosA*AFcosA);

        float2 BF = input.Pos.xy - input.EdgeB.xy;
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

float4 PsSolidWire( PS_INPUT_WIRE input) : SV_Target
{
    // Compute the shortest distance between the fragment and the edges.
    float dist = evalMinDistanceToEdges(input);

    // Map the computed distance to the [0,2] range on the border of the line.
    dist = clamp((dist - (0.5*LineWidth - 1)), 0, 2);

    // Alpha is computed from the function exp2(-2(x)^2).
    dist *= dist;
    float alpha = exp2(-2*dist);


    float3 amb = float3(0.05, 0.05, 0.05);
    float dff = saturate(dot(-SUN_DIR, input.normal));
    float3 col = amb + dff * float3(0.1, 0.1, 0.25);

    col = lerp(col, WireColor.rgb, alpha);

    float b = 0.001;
    float fogAmount = max(0, 1 - exp(-(input.distance - 500) * b));

    float3 fogColor = FogColor(input.rayDir);
    return float4(lerp(col, fogColor, fogAmount), 0.9);
}


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

//------------------------------------------------------
// boids
//------------------------------------------------------
VsBoidsOut VsBoids(VsBoidsIn input)
{
  VsBoidsOut output;
  float4x4 mtxWorldViewProj = mul(world, viewProj);
  output.pos = mul(float4(input.pos, 1), mtxWorldViewProj);
  output.normal = mul(float4(input.normal, 0), world).xyz;
  output.normal = input.normal;
  return output;
}

float4 PsBoids(VsBoidsOut p) : SV_Target
{
  return BOID_COLOR * max(0,(dot(normalize(p.normal), float3(0,-1,0))));
}

//------------------------------------------------------
// composite
//------------------------------------------------------
float4 PsComposite(VSQuadOut p) : SV_Target
{
  float2 uv = p.uv.xy;
  float2 xx = -1 + 2 * uv;

  float4 backgroundCol = Texture0.Sample(PointSampler, uv);

   // gamma correction
  float4 color = pow(backgroundCol, 1.0/2.2);

  float r = 0.7 + 0.9 - smoothstep(0, 1, sqrt(xx.x*xx.x + xx.y*xx.y));
  return r * color;
}

