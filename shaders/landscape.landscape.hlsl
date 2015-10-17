#include "landscape.hlsl"

cbuffer G : register(b0)
{
  float4 dim;
};

cbuffer V : register(b0)
{
  matrix world;
  matrix viewProj;
  float3 cameraPos;
  float4 musicParams;
};

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

// entry-point: vs
VsLandscapeOut VsLandscape(VsLandscapeIn v)
{
  VsLandscapeOut res;
  matrix worldViewProj = mul(world, viewProj);

  // This is the biggest cheese, but if I don't do it, everything goes to hell..
  float3 pos = v.pos;
  if (pos.z == cameraPos.z)
    pos.z += 0.01;

  pos.y *= (1 + musicParams.y / 5);

  res.pos = mul(float4(pos, 1), worldViewProj);
  res.normal = mul(float4(v.normal, 0), world).xyz;
  float3 dir = pos - cameraPos;
  res.distance = length(dir);
  res.rayDir = dir * 1/res.distance;
  return res;
}

float2 projToWindow(in float4 pos)
{
    return float2(  dim.x*0.5*((pos.x/pos.w) + 1) + dim.z,
                    dim.y*0.5*(1-(pos.y/pos.w)) + dim.w );
}


[maxvertexcount(3)]
// entry-point: gs
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

static float4 WireColor = float4(1, 1, 1, 1);
static float LineWidth = 2;

// entry-point: ps
PsColBrightnessOut PsLandscape(PsLandscapeIn p)
{
    PsColBrightnessOut res;

    // Compute the shortest distance between the fragment and the edges.
    float dist = MainDistanceToEdge(p);
    //return ColorCases[p.Case];

    // Map the computed distance to the [0,2] range on the border of the line.
    dist = clamp((dist - (0.5*LineWidth - 1)), 0, 3);

    // Alpha is computed from the function exp2(-2(x)^2).
    dist *= dist;
    float alpha = exp2(-2*dist);

    float3 amb = float3(0.01, 0.01, 0.01);
    float dff = saturate(dot(-SUN_DIR, p.normal));

    float3 diffuseCol = float3(130, 45, 130) / 255;

    float3 col = (amb + dff) * diffuseCol;

    col = lerp(col, WireColor.rgb, alpha);

    float fogAmount = max(0, 1 - exp(-(p.distance) * FOG_SCALE));
    float3 fogColor = FogColor(p.rayDir);
    col = lerp(col, fogColor, fogAmount);

    res.col = float4(col, 0.9);
    float lum = Luminance(col.rgb);
    res.emissive.rgb = 0;
    res.emissive.a = smoothstep(0.3, 1, 2 * lum);
    return res;
}
