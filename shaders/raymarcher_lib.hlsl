// from http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
// http://www.iquilezles.org/www/articles/smin/smin.htm	

float sdSphere( float3 p, float s )
{
  return length(p)-s;
}
		
float udBox( float3 p, float3 b )
{
  // abs(p) is used for symmetry
  // p-b is the distance from p to the edge of the box in each axis
  // max is used to guarantee non-negative
  return length(max(abs(p)-b,0.0));
}

float udRoundBox( float3 p, float3 b, float r )
{
  return length(max(abs(p)-b,0.0))-r;
}
		
float sdBox( float3 p, float3 b )
{
	// the test has 2 parts:
	// - the signed part: max(d.x, max(d.y,d.z)), which is the largest negative penetration
	// - the unsigned part: same as udBox
  float3 d = abs(p) - b;
  return min(max(d.x, max(d.y,d.z)), 0.0) + length(max(d,0.0));
}

float sdTorus( float3 p, float2 t )
{
  float2 q = float2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}
		
float sdCylinder( float3 p, float3 c )
{
  return length(p.xz-c.xy)-c.z;
}

float sdCone( float3 p, float2 c )
{
    // c must be normalized
    float q = length(p.xy);
    return dot(c,float2(q,p.z));
}
		
float sdPlane( float3 p, float4 n )
{
  // n must be normalized
  return dot(p,n.xyz) + n.w;
}

float sdHexPrism( float3 p, float2 h )
{
    float3 q = abs(p);
    return max(q.z-h.y,max((q.x*0.866025+q.y*0.5),q.y)-h.x);
}
		
float sdTriPrism( float3 p, float2 h )
{
    float3 q = abs(p);
    return max(q.z-h.y,max(q.x*0.866025+p.y*0.5,-p.y)-h.x*0.5);
}

float sdCapsule( float3 p, float3 a, float3 b, float r )
{
    float3 pa = p - a, ba = b - a;
    float h = clamp( dot(pa,ba)/dot(ba,ba), 0.0, 1.0 );
    return length( pa - ba*h ) - r;
}
		
float sdCappedCylinder( float3 p, float2 h )
{
  float2 d = abs(float2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float dot2( in float3 v ) { return dot(v,v); }
float udTriangle( float3 p, float3 a, float3 b, float3 c )
{
    float3 ba = b - a; float3 pa = p - a;
    float3 cb = c - b; float3 pb = p - b;
    float3 ac = a - c; float3 pc = p - c;
    float3 nor = cross( ba, ac );

    return sqrt(
    (sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(ac,nor),pc))<2.0)
     ?
     min( min(
     dot2(ba*clamp(dot(ba,pa)/dot2(ba),0.0,1.0)-pa),
     dot2(cb*clamp(dot(cb,pb)/dot2(cb),0.0,1.0)-pb) ),
     dot2(ac*clamp(dot(ac,pc)/dot2(ac),0.0,1.0)-pc) )
     :
     dot(nor,pa)*dot(nor,pa)/dot2(nor) );
}
		
float udQuad( float3 p, float3 a, float3 b, float3 c, float3 d )
{
    float3 ba = b - a; float3 pa = p - a;
    float3 cb = c - b; float3 pb = p - b;
    float3 dc = d - c; float3 pc = p - c;
    float3 ad = a - d; float3 pd = p - d;
    float3 nor = cross( ba, ad );

    return sqrt(
    (sign(dot(cross(ba,nor),pa)) +
     sign(dot(cross(cb,nor),pb)) +
     sign(dot(cross(dc,nor),pc)) +
     sign(dot(cross(ad,nor),pd))<3.0)
     ?
     min( min( min(
     dot2(ba*clamp(dot(ba,pa)/dot2(ba),0.0,1.0)-pa),
     dot2(cb*clamp(dot(cb,pb)/dot2(cb),0.0,1.0)-pb) ),
     dot2(dc*clamp(dot(dc,pc)/dot2(dc),0.0,1.0)-pc) ),
     dot2(ad*clamp(dot(ad,pd)/dot2(ad),0.0,1.0)-pd) )
     :
     dot(nor,pa)*dot(nor,pa)/dot2(nor) );
}

float sdCappedCone( in float3 p, in float3 c )
{
    float2 q = float2( length(p.xz), p.y );
    float2 v = float2( c.z*c.y/c.x, -c.z );

    float2 w = v - q;

    float2 vv = float2( dot(v,v), v.x*v.x );
    float2 qv = float2( dot(v,w), v.x*w.x );

    float2 d = max(qv,0.0)*qv/vv;

    return sqrt( dot(w,w) - max(d.x,d.y) )* sign(max(q.y*v.x-q.x*v.y,w.y));
}
			

// Union
float opU( float d1, float d2 )
{
    return min(d1,d2);
}
		
// Substraction
float opS( float d1, float d2 )
{
    return max(-d1,d2);
}
		
// Intersection
float opI( float d1, float d2 )
{
    return max(d1,d2);
}

// polynomial smooth min (k = 0.1);
float smin( float a, float b, float k )
{
    float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
    return lerp( b, a, h ) - k*h*(1.0-h);
}