#pragma once
#ifndef SMOOTHDRIVER_H
#define SMOOTHDRIVER_H

/***************

SmoothDriver v1.0
by cbloom

two Drive functions are the basic entry points :
	take a smooth (C1) step towards a target point and velocity
	gauranteed to converge in finite time, proportional to coverge_time, subject to constraints

see writings on my site for more information

this header works on 3d values, but extension to N dimensions is trivial

1d is actually a lot simpler, but I don't provide the simple 1d case at the moment, maybe next time

*****************/

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "tano_math.hpp"

namespace SmoothDriver
{
  typedef tano::vec3 Vec3;

//=================================================================================

  
//class Vec3; // Vec3 should be a float[3]
	// presumably you have your own Vec3 type, so you can just cast to SmoothDriver::Vec3 when you call these

// cubic curve with max accel
//	max_accel should be somewhere around (typical_distance/coverge_time^2)
void DriveCubic(Vec3 * pos,Vec3 * vel,const Vec3 * toPos,const Vec3 * toVel,
		float max_accel,float time_step);

//	PD damping should generally be between 1.0 and 1.1
//	frequency should be like 1.0/coverge_time
void DrivePDClamped(Vec3 * pos,Vec3 * vel,const Vec3 * toPos,const Vec3 * toVel,
			const float frequency,
			const float damping,
			const float minVelocity,
			float time_step);

//=================================================================================

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#define EPSILON		(0.00001f)

#define ASSERT(exp)		assert(exp)

#define MIN(a,b)	( (a) < (b) ? (a) : (b) )
#define MAX(a,b)	( (a) > (b) ? (a) : (b) )

inline float fsquare(const float f)
{
	return f*f;
}

//! fsign; returns 1.f or -1.f for positive or negative float
//! could do a much faster version using float_AS_INT if this is needed
inline float fsign(const float f)
{
	return (f >= 0.f) ? 1.f : -1.f;
}

//! float == 0.f test with slop
inline bool fiszero(const float f1,const float tolerance = EPSILON)
{
	return fabsf(f1) <= tolerance;
}
//! float == 0.f test with slop
inline bool fiszero(const double f1,const double tolerance = EPSILON)
{
	return fabs(f1) <= tolerance;
}

//! return a Clamp float in the range lo to hi
inline float fclamp(const float x,const float lo,const float hi)
{
	return ( ( x < lo ) ? lo : ( x > hi ) ? hi : x );
}

//-----------------------------------------------------------

//! complex number class
class ComplexDouble
{
public:
	double re,im;

	//-------------------------------------------------------------------

	ComplexDouble()
	{
	}

	explicit ComplexDouble(double r) : re(r), im(0.0)
	{
	}

	explicit ComplexDouble(double r,double i) : re(r) , im(i)
	{
	}
	
	static const ComplexDouble zero;
	//static const ComplexDouble one;
	//static const ComplexDouble I;

	void operator = (const double v);
	void operator *= (const double scale);
	void operator /= (const double scale);
	void operator *= (const ComplexDouble & v);
	void operator /= (const ComplexDouble & v);
	void operator += (const double val);
	void operator -= (const double val);
	void operator += (const ComplexDouble & v);
	void operator -= (const ComplexDouble & v);
	bool operator == (const ComplexDouble & v) const; //!< exact equality test
	bool operator != (const ComplexDouble & v) const; //!< exact equality test
	bool operator == (const double v) const; //!< exact equality test
	bool operator != (const double v) const; //!< exact equality test

	static const ComplexDouble MakeUnitPolar(const double radians)
	{
		double c = cos(radians);
		double s = sin(radians);
		return ComplexDouble(c,s);
	}

	static const ComplexDouble MakePolar(const double mag,const double radians)
	{
		double c = cos(radians);
		double s = sin(radians);	
		return ComplexDouble(mag*c,mag*s);
	}
	
	const ComplexDouble GetConjugate() const
	{
		return ComplexDouble(re,-im);
	}
	
	double GetMagnitudeSquared() const
	{
		return re*re + im*im;
	}
	double GetMagnitude() const
	{
		return sqrt( GetMagnitudeSquared() );
	}
	double GetAngle() const
	{
		return atan2( im, re );
	}
	
	const ComplexDouble GetInverse() const
	{
		double m2 = GetMagnitudeSquared();
		ASSERT( m2 != 0.0 );
		double inv = 1.0/m2;
		return ComplexDouble(re*inv,-im*inv);
	}
	
	const ComplexDouble GetNormalized() const
	{
		double mag = GetMagnitude();
		ASSERT( mag != 0.0 );
		double inv = 1.0/mag;
		return ComplexDouble(re*inv,im*inv);
	}
	
	bool IsReal(const double tolerance = EPSILON) const
	{
		return fiszero(im,tolerance);
	}
	bool Equals(const ComplexDouble & rhs,const double tolerance = EPSILON) const
	{
		return	fiszero( re - rhs.re , tolerance ) &&
				fiszero( im - rhs.im , tolerance );
	}
	bool Equals(const double rhs,const double tolerance = EPSILON) const
	{
		return	fiszero( re - rhs , tolerance ) &&
				fiszero( im , tolerance );
	}
};

inline void ComplexDouble::operator = (const double v)
{
	re = v;
	im = 0.0;
}

inline void ComplexDouble::operator *= (const double scale)
{
	re *= scale;
	im *= scale;
}

inline void ComplexDouble::operator *= (const ComplexDouble & rhs)
{
	double r = re * rhs.re - im * rhs.im;
	double i = re * rhs.im + im * rhs.re;
	re = r;
	im = i;
}

inline void ComplexDouble::operator /= (const double scale)
{
	ASSERT( scale != 0.f );
	const double inv = 1.f / scale;
	re *= inv;
	im *= inv;
}

inline void ComplexDouble::operator /= (const ComplexDouble & rhs)
{
	(*this) *= rhs.GetInverse();
}

inline void ComplexDouble::operator += (const ComplexDouble & v)
{
	re += v.re;
	im += v.im;
}

inline void ComplexDouble::operator -= (const ComplexDouble & v)
{
	re -= v.re;
	im -= v.im;
}

inline void ComplexDouble::operator += (const double v)
{
	re += v;
}

inline void ComplexDouble::operator -= (const double v)
{
	re -= v;
}

inline bool ComplexDouble::operator == (const ComplexDouble & v) const
{
	return (re == v.re &&
			im == v.im );
}

inline bool ComplexDouble::operator == (const double v) const
{
	return (re == v &&
			im == 0.0 );
}

inline bool ComplexDouble::operator != (const ComplexDouble & v) const
{
	return !operator==(v);
}

inline bool ComplexDouble::operator != (const double v) const
{
	return (re != v ||
			im != 0.0 );
}

inline const ComplexDouble operator - (const ComplexDouble & a )
{
	return ComplexDouble( - a.re, - a.im );
}

inline const ComplexDouble operator + (const ComplexDouble & a )
{
	return a;
}

inline const ComplexDouble operator * (const ComplexDouble & a,const ComplexDouble & b)
{
	return ComplexDouble( a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re );
}

inline const ComplexDouble operator / (const ComplexDouble & a,const ComplexDouble & b)
{
	const ComplexDouble binv = b.GetInverse();
	return a * binv;
}

inline const ComplexDouble operator + (const ComplexDouble & a,const ComplexDouble & b)
{
	return ComplexDouble( a.re + b.re, a.im + b.im );
}

inline const ComplexDouble operator - (const ComplexDouble & a,const ComplexDouble & b)
{
	return ComplexDouble( a.re - b.re, a.im - b.im );
}

inline const ComplexDouble operator * (const ComplexDouble & a,const double b)
{
	return ComplexDouble( a.re * b, a.im * b );
}

inline const ComplexDouble operator / (const ComplexDouble & a,const double b)
{
	return ComplexDouble( a.re / b , a.im / b );
}

inline const ComplexDouble operator + (const ComplexDouble & a,const double b)
{
	return ComplexDouble( a.re + b, a.im );
}

inline const ComplexDouble operator - (const ComplexDouble & a,const double b)
{
	return ComplexDouble( a.re - b, a.im );
}

inline const ComplexDouble operator * (const double b,const ComplexDouble & a)
{
	return ComplexDouble( a.re * b, a.im * b );
}

inline const ComplexDouble operator / (const double b,const ComplexDouble & a)
{
	return ComplexDouble(b,0) / a;
}

inline const ComplexDouble operator + (const double b,const ComplexDouble & a)
{
	return ComplexDouble( a.re + b, a.im );
}

inline const ComplexDouble operator - (const double b,const ComplexDouble & a)
{
	return ComplexDouble( b - a.re, - a.im );
}

inline const ComplexDouble ComplexPow(const ComplexDouble & v, const double p)
{
	double m = v.GetMagnitude();
	double a = v.GetAngle();
	
	double mp = pow(m,p);
	double ma = a*p;
	
	return ComplexDouble::MakePolar(mp,ma);
}

inline const ComplexDouble ComplexPow(const double v, const double p)
{
	double m,a;
	
	if ( v >= 0 )
	{
		m = v;
		a = 0; // or 2PI , or 4PI , etc.
	}
	else
	{
		m = -v;
		a = M_PI;
	}
	
	double mp = pow(m,p);
	double ma = a*p;
	
	return ComplexDouble::MakePolar(mp,ma);	
}
					
inline const ComplexDouble ComplexSqrt(const ComplexDouble & v)
{
	return ComplexPow(v,0.5);
}

inline const ComplexDouble ComplexSqrt(const double v)
{
	if ( v >= 0 )
	{
		return ComplexDouble( sqrt(v) , 0 );
	}
	else
	{
		return ComplexDouble( 0 , sqrt(-v) );
	}
}

inline void SolveQuadratic(const double a,const double b,const double c,
					ComplexDouble * root1,ComplexDouble * root2)
{
	// ( -b +/- sqrt( b*b - 4*a*c )) / 2a
	
	double inside = b*b - 4.0*a*c;
	ComplexDouble sqrtpart = ComplexSqrt( inside );
	sqrtpart /= (2.0*a);
	double realpart = -b / (2.0*a);
	 
	*root1 = realpart + sqrtpart;
	*root2 = realpart - sqrtpart;
}


inline void SolveCubic(double a,double b,double c,double d,
					ComplexDouble * px0,
					ComplexDouble * px1,
					ComplexDouble * px2)
{
	ASSERT( a != 0.0 );
	
	double q = (3.0*a*c - b*b)/(9.0*a*a);
	double r = (9.0*a*b*c - 27.0*a*a*d - 2.0*b*b*b)/(54.0*a*a*a);
	
	ComplexDouble u = ComplexSqrt( q*q*q + r*r );
	
	ComplexDouble s = ComplexPow( r + u, 1.0/3.0 ); 
	ComplexDouble t = ComplexPow( r - u, 1.0/3.0 );
	
	*px0 = s + t - b/(3.0*a);

	ComplexDouble v1 = -0.5*(s + t) - b/(3.0*a);
	ComplexDouble v2 = 0.5*sqrt(3.0)*(s - t)*ComplexDouble(0.f,1.f);
	
	*px1 = v1 + v2;

	*px2 = v1 - v2;
}

inline void SolveDepressedQuartic(double A,double C,double D,double E,
					ComplexDouble * px0,
					ComplexDouble * px1,
					ComplexDouble * px2,
					ComplexDouble * px3)
{
	// try to solve the quartic analytically and get all the roots
	
	ASSERT( A != 0.0 );
	
	// if E == 0 there is one root at 0, and then we solve a depressed cubic
	
	if ( E == 0.0 )
	{
		*px3 = 0.0;
		
		SolveCubic(A,0.0,C,D,px0,px1,px2);
		return;
	}
	
	if ( D == 0.0 )
	{
		// just solve a quadratic for t^2
		
		ComplexDouble r1,r2;
		SolveQuadratic(A,C,E,&r1,&r2);
		
		*px0 = ComplexSqrt( r1 );
		*px1 = - *px0;
		*px2 = ComplexSqrt( r2 );
		*px3 = - *px2;

		return;
	}

	// if C == 0 ??	

	double alpha = C/A;
	double beta = D/A;
	double gamma = E/A;
	
	double P = - alpha*alpha/12.0 - gamma;
	double Q = - (alpha*alpha*alpha)/108.0 + (alpha*gamma)/3.0 - (beta*beta)/8.0;
	double forR = Q*Q/4.0 + P*P*P/27.0;
	ComplexDouble R = (Q/2.0) + ComplexSqrt( forR );
	
	ComplexDouble U = ComplexPow(R,1.0/3.0);
	
	ComplexDouble y( - (5.0/6.0)*alpha );
	if ( U != 0.0 )
		y += -U + P/(3.0*U);
	
	ComplexDouble W = ComplexSqrt( alpha + 2.0*y );
	
	ASSERT( W != 0.0 ); // possible ?
	
	ComplexDouble p1 = 3.0*alpha + 2.0*y;
	ComplexDouble p2 = 2.0*beta/W;
	
	ComplexDouble sqrtpos = ComplexSqrt( - (p1 + p2) );
	ComplexDouble sqrtneg = ComplexSqrt( - (p1 - p2) );
	
	*px0 = 0.5 * ( + W + sqrtpos );
	*px1 = 0.5 * ( - W + sqrtneg );
	*px2 = 0.5 * ( + W - sqrtpos );
	*px3 = 0.5 * ( - W - sqrtneg );
}

//-----------------------------------------------------------

//! 3-Space Vector
#if 0
class Vec3
{
public:
	//! Vec3 is just a bag of data;
	float x,y,z;

	inline float LengthSqr() const
	{
		return x * x + y * y + z * z;
	}

	inline float Length() const
	{
		return sqrtf( LengthSqr() );
	}
	
	__forceinline Vec3() { }
	
	__forceinline Vec3(const float ix,const float iy,const float iz) : 
		x(ix), y(iy), z(iz)
	{
	}
	
	inline void operator *= (const float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
	}
	
	inline bool operator == (const Vec3 & other) const
	{
		return memcmp(this,&other,sizeof(Vec3)) == 0;
	}
};

__forceinline float operator * (const Vec3 & u,const Vec3 & v) // dot
{
	return ( u.x * v.x + u.y * v.y + u.z * v.z );
}

__forceinline const Vec3 operator - (const Vec3 & v) // negative
{
	return Vec3( - v.x , - v.y , - v.z );
}

__forceinline const Vec3 operator * (const float f,const Vec3 & v) // scale
{
	return Vec3( f * v.x , f * v.y , f * v.z );
}

__forceinline const Vec3 operator * (const Vec3 & v,const float f) // scale
{
	return Vec3( f * v.x , f * v.y , f * v.z );
}


__forceinline const Vec3 operator / (const Vec3 & v,const float f) // scale
{
	ASSERT( f != 0.f );
	const float inv = 1.f / f;
	return Vec3( inv * v.x , inv * v.y , inv * v.z );
}

__forceinline const Vec3 operator + (const Vec3 & u,const Vec3 & v) // add
{
	return Vec3( u.x + v.x , u.y + v.y , u.z + v.z );
}

__forceinline const Vec3 operator - (const Vec3 & u,const Vec3 & v) // subtract
{
	return Vec3( u.x - v.x , u.y - v.y , u.z - v.z );
}

#endif

//Cross product.
__forceinline const Vec3 operator ^ (const Vec3 & u,const Vec3 & v)
{
  return Vec3( 
    u.y * v.z - u.z * v.y,
    u.z * v.x - u.x * v.z,
    u.x * v.y - u.y * v.x );
}

__forceinline float DistanceSqr(const Vec3 & a,const Vec3 &b)
{
	return fsquare(a.x - b.x) + fsquare(a.y - b.y) + fsquare(a.z - b.z);
}
	
__forceinline void ClampToMinLen(Vec3 * pVec,const float minLen)
{
	const float lenSqr = LengthSquared(*pVec);
	if ( lenSqr < fsquare(minLen) )
	{
		// can be problematic :
		if ( lenSqr == 0.f )
			return;
		*pVec *= (minLen / sqrtf(lenSqr));
	}
}
	
//-----------------------------------------------------------

/*
	damping is usually 1.0 (critically damped)
	frequency is usually in the range 1..16
		freq has units [1/time]
		damping is unitless
		
	ComputePDAccel is NOT an exact step, it's an implicit Euler step,
	which is more stable and has some artificial damping
	
	It should be used to step velocity first, then step pos with the new velocity :
	float v1 = v0 + a*dt;
	float x1 = x0 + v1 * dt;
*/
inline float ComputePDAccel(const float fmValue,const float fmVelocity,
					const float toValue,const float toVelocity,
					const float frequency, const float damping,
					const float dt )
{
	const float ks = fsquare(frequency)*(36.f);
	const float kd = frequency*damping*(9.f);

	// scale factor for implicit integrator :
	//	usually just slightly less than one
	const float scale = 1.0f / ( 1.0f  + kd* dt + ks*dt*dt );

	const float ksI = ks  *  scale;
	const float kdI = ( kd + ks* dt ) * scale;

	return ksI*(toValue-fmValue) + kdI*(toVelocity-fmVelocity);
}

inline void ApplyPDClamping(
					float * pValue,
					float * pVelocity,
					const float toValue,
					const float toVelocity,
					const float frequency,
					const float damping,
					const float minVelocity,
					const float dt )
{
	float x0 = *pValue;
	float v0 = *pVelocity;

	float a = ComputePDAccel(x0,v0,toValue,toVelocity,frequency,damping,dt);
	
	// step value with the velocity from the end of the frame :
	float v1 = v0 + a*dt;
	float x1 = x0 + v1 * dt;

	float smallStep = minVelocity * dt;

	if ( fabsf(x1-x0) <= smallStep )
	{
		if ( fabsf(x0 - toValue) <= smallStep )
		{
			*pValue = toValue;
			*pVelocity = toVelocity;
		}
		else
		{
			// tiny velocities, do min step
			float sign = fsign(toValue - x0); 
			*pVelocity = sign * minVelocity;
			*pValue = x0 + (*pVelocity) * dt;
		}
	}
	else
	{
		*pValue = x1;
		*pVelocity = v1;
	}
	
}

inline void ApplyPDClamping(
					Vec3 * pValue,
					Vec3 * pVelocity,
					const Vec3 toValue,
					const Vec3 toVelocity,
					const float frequency,
					const float damping,
					const float minVelocity,
					const float dt )
{
	Vec3 x0 = *pValue;
	Vec3 v0 = *pVelocity;

	Vec3 a;

	a.x = ComputePDAccel(x0.x ,v0.x ,toValue.x ,toVelocity.x ,frequency,damping,dt);
	a.y = ComputePDAccel(x0.y ,v0.y ,toValue.y ,toVelocity.y ,frequency,damping,dt);
	a.z = ComputePDAccel(x0.z ,v0.z ,toValue.z ,toVelocity.z ,frequency,damping,dt);
	
	// step value with the velocity from the end of the frame :
	Vec3 v1 = v0 + a*dt;
	Vec3 x1 = x0 + v1 * dt;

	float smallStep = minVelocity * dt;

	if ( DistanceSqr(x1,x0) <= fsquare(smallStep) )
	{
		if ( DistanceSqr(x0,toValue) <= fsquare(smallStep) )
		{
			*pValue = toValue;
			*pVelocity = toVelocity;
		}
		else
		{
			// tiny velocities, do min step
			*pVelocity = toValue - x0;
			ClampToMinLen(pVelocity,minVelocity);
			*pValue = x0 + (*pVelocity) * dt;
		}
	}
	else
	{
		*pValue = x1;
		*pVelocity = v1;
	}
	
}
					
struct Bezier
{
	Vec3 B0,B1,B2,B3;
	
	void Fit(const Vec3 &x0, const Vec3 &x1, const Vec3 &v0, const Vec3 &v1, const float T)
	{
		// bezier controls :
		B0 = x0;
		B3 = x1;
		//d0 = 3*(B1-B0);
		B1 = x0 + v0*(T/3.f);
		B2 = x1 - v1*(T/3.f);
	}
	
	const Vec3 X(const float t) const
	{
		const float s = 1.f - t;
		return B0* (s*s*s) + B1*(s*s*t*3.f) + B2*(t*t*s*3.f) + B3*(t*t*t);
	}
	
	const Vec3 dX(const float t) const
	{
		const float s = 1.f - t;
		return (B1-B0) * (3.f*s*s) + (B2-B1) * (6.f*s*t) + (B3 - B2) * (3.f*t*t);
	}
	
	const Vec3 ddX(const float t) const
	{
		const Vec3 A0 = 6.f*B0 + 6.f*B2 - 12.f*B1;
		const Vec3 A1 = 3.f*B1 - 3.f*B2 - B0 + B3;
		
		return A0 + (6.f*t) * A1;
	}
	
};


// how close are we to having the max accel be what we want :
inline float ErrorBezierWithMaxAccel(const Vec3 &x0, const Vec3 &x1, const Vec3 &v0, const Vec3 &v1, const float T, const float maxA)
{
	Bezier bez;
	bez.Fit(x0,x1,v0,v1,T);
	
	float a0 = bez.ddX(0).Length()/fsquare(T);
	float a1 = bez.ddX(1).Length()/fsquare(T);
	float a = MAX(a0,a1);
	
	return fsquare( a - maxA );
}

inline float MakeBezierWithMaxAccel_BinSearch(Bezier * pBez,const Vec3 &x0, const Vec3 &x1, const Vec3 &v0, const Vec3 &v1, const float minT, const float maxA)
{
	/**
	
	CB note : there's an alternative way to do this, you can solve it analytically.
	It's a quartic equation in T, so you then have to use Newton's Method or something to solve for the roots of the quartic.
	
	**/

	// first see if minT gives us a low enough acceleration :

	pBez->Fit(x0,x1,v0,v1,minT);
	
	float a0 = pBez->ddX(0).Length()/fsquare(minT);
	float a1 = pBez->ddX(1).Length()/fsquare(minT);

	if ( a0 < maxA && a1 < maxA )
	{
		return minT;
	}
	
	// no ; well then find a bound where loT is too short and hiT is too long :
	
	float loT = minT;
		
	for(;;)
	{
		float hiT = loT*2;
	
		pBez->Fit(x0,x1,v0,v1,hiT);
		
		float a0 = pBez->ddX(0).Length()/fsquare(hiT);
		float a1 = pBez->ddX(1).Length()/fsquare(hiT);
	
		if ( a0 < maxA && a1 < maxA )
		{
			break;
		}
		
		loT = hiT;
	}
	
	// somewhere between loT and hiT is good.
	// binary search

	float curT = loT*1.5f; //faverage(loT,loT*2);
	float Tstep = loT*0.5f;
	
	float curErr = ErrorBezierWithMaxAccel(x0,x1,v0,v1,curT,maxA);
		
	for(int i=0;i<16;i++)
	{
		float upT = curT + Tstep;
		float dnT = curT - Tstep;
	
		float upErr = ErrorBezierWithMaxAccel(x0,x1,v0,v1,upT,maxA);
		float dnErr = ErrorBezierWithMaxAccel(x0,x1,v0,v1,dnT,maxA);

		if ( upErr < curErr && upErr < dnErr )
		{
			curT = upT;
		}		
		else if ( dnErr < curErr )
		{
			curT = dnT;
		}
		
		Tstep *= 0.5f;
	}
	
	pBez->Fit(x0,x1,v0,v1,curT);
	return curT;
}

struct QuarticRoots
{
	double data[4];
	int	count;
	
	QuarticRoots() : count(0) { }
	
	void clear() { count = 0; }
	void push_back(double d) { data[count++] = d; }
	int size() const { return count; }
	double operator [] (const int i) { return data[i]; }
};

inline void SolveQuarticReal(QuarticRoots * roots,double A,double C,double D,double E)
{
	/*
	// try to solve the quartic analytically and get all the roots
	// I'm doing something evil and just absing the sqrts to avoid imaginaries

	double alpha = C/A;
	double beta = D/A;
	double gamma = E/A;
	
	double P = - alpha*alpha/12.0 - gamma;
	double Q = - alpha*alpha*alpha/108.0 + alpha*gamma/3.0 - beta*beta/8.0;
	double R = Q/2.0 + sqrt( fabs( Q*Q/4.0 + P*P*P/27.0 ) );
	
	double U = pow(fabs(R),1.0/3.0);
	
	double y = - (5.0/6.0)*alpha - U + P/(3.0*U);
	
	double W = sqrt( fabs( alpha + 2.0*y ) );
	
	double x0 = 0.5 * ( + W + sqrt( fabs( 3.0*alpha + 2.0*y + 2.0*beta/W ) ) );
	double x1 = 0.5 * ( - W + sqrt( fabs( 3.0*alpha + 2.0*y - 2.0*beta/W ) ) );
	double x2 = 0.5 * ( + W - sqrt( fabs( 3.0*alpha + 2.0*y + 2.0*beta/W ) ) );
	double x3 = 0.5 * ( - W - sqrt( fabs( 3.0*alpha + 2.0*y - 2.0*beta/W ) ) );
	
	*/

	ComplexDouble x0,x1,x2,x3;
	SolveDepressedQuartic(A,C,D,E,&x0,&x1,&x2,&x3);	
	
	// one or more of these solutions may be bogus if they should've been imaginary
	//	so, check them to make sure they are real solutions :
	
	roots->clear();
	
	if ( x0.IsReal() ) roots->push_back(x0.re);
	if ( x1.IsReal() ) roots->push_back(x1.re);
	if ( x2.IsReal() ) roots->push_back(x2.re);
	if ( x3.IsReal() ) roots->push_back(x3.re);
	
	return;
}

inline double SolveQuarticNonNegative(double A,double C,double D,double E)
{
	QuarticRoots vals;
	SolveQuarticReal(&vals,A,C,D,E);
		
	// return the lowest non-negative solution :
	
	const double badval = 99999.f;
	double ret = badval;
	
	for(int i=0;i<vals.size();i++)
	{
		if ( vals[i] > 0 && vals[i] < ret )
			ret = vals[i];
	}
	
	// there should be one !!
	ASSERT( ret != badval );
	
	return ret;
}

inline float MakeBezierWithMaxAccel_Quartic(Bezier * pBez,const Vec3 &x0, const Vec3 &x1, const Vec3 &v0, const Vec3 &v1, const float minT, const float maxA)
{
	// first see if minT gives us a low enough acceleration :

	pBez->Fit(x0,x1,v0,v1,minT);
	
	float a0 = pBez->ddX(0).Length()/fsquare(minT);
	float a1 = pBez->ddX(1).Length()/fsquare(minT);

	if ( a0 < maxA && a1 < maxA )
	{
		return minT;
	}

	/*
	
	CB note : BTW in 1d you can just solve a quadratic which is way easier
	the quartic comes up because we have to square the quadratic to turn the vectors into lengths
	
	*/
	
	Vec3 xd = x1 - x0;
	
	double A = maxA*maxA / 36.0;
	double D = - xd.LengthSqr();
	
	Vec3 vvv0 = v0 + v0 + v1;
	double B0 = (-1.0/9.0)* vvv0.LengthSqr();
	double C0 = (2.0/3.0) * ( Dot(xd, vvv0) );

	Vec3 vvv1 = v1 + v1 + v0;
	double B1 = (-1.0/9.0)* vvv1.LengthSqr();
	double C1 = (2.0/3.0) * ( Dot(xd, vvv1) );
	
	// start at minT so we find the lowest root of the quartic

	double t0 = SolveQuarticNonNegative(A,B0,C0,D);
	double t1 = SolveQuarticNonNegative(A,B1,C1,D);
	
	float t = (float) MAX(t0,t1);
		
	pBez->Fit(x0,x1,v0,v1,t);
	
	a0 = pBez->ddX(0).Length()/fsquare(t);
	a1 = pBez->ddX(1).Length()/fsquare(t);
	
	return t;
}

//===========================================================================

// cubic curve with max accel
//	max_accel should be somewhere around (typical_distance/coverge_time^2)
inline void DriveCubic(Vec3 * pos,Vec3 * vel,const Vec3 * toPos,const Vec3 * toVel,
		float max_accel,float time_step)
{
	// solve cubic
	Bezier curve;
	
	// these both work :			
	// float T = MakeBezierWithMaxAccel_BinSearch(&curve,
	float T = MakeBezierWithMaxAccel_Quartic(&curve,
		*pos,
		*toPos,
		*vel,
		*toVel,
		time_step,
		max_accel);
	
	// step along by dt :
	float t = time_step / T;
			
	if ( t >= 1.f - EPSILON )
	{
		*pos = *toPos;
		*vel = *toVel;
	}
	else
	{				
		*pos = curve.X(t);
		*vel = curve.dX(t) * (1.f / T);
	}
}

/*
	PD damping should generally be between 1.0 and 1.1
	frequency should be like 1.0/coverge_time
*/
inline void DrivePDClamped(Vec3 * pos,Vec3 * vel,const Vec3 * toPos,const Vec3 * toVel,
			const float frequency,
			const float damping,
			const float minVelocity,
			float time_step)
{
	ApplyPDClamping(pos,vel,*toPos,*toVel,frequency,damping,minVelocity,time_step);
}					

//-------------------------------------------------------------------------
// "Hybrid" cubic is like max accel, but is gauranteed to reach a stationary target in max_time
//	C1 smooth

struct HybridCubicState
{
	Vec3	toPos,toVel;
	double	time_remaining;
	
	HybridCubicState() : time_remaining(-1.f) { }

	void SetTarget(const Vec3 & _toPos,const Vec3 & _toVel)
	{
		if ( toPos == _toPos && toVel == _toVel )
		{
		}
		else
		{
			toPos = _toPos;
			toVel = _toVel;
			time_remaining = -1.f;
		}
	}	
};

// cubic curve with max accel
//	max_accel should be somewhere around (typical_distance/coverge_time^2)
inline void DriveCubicHybrid(Vec3 * pos,Vec3 * vel,HybridCubicState * pState,
		float max_accel,float max_time,float time_step)
{
	// hybrid cubic :
	// max time is set from max_time
	// min time is set by maxaccel

	if ( pState->time_remaining < 0.f )
	{
		// solve cubic
		Bezier curve;
		
		// these both work :			
		// float T = MakeBezierWithMaxAccel_BinSearch(&curve,
		float maxaccel_time = MakeBezierWithMaxAccel_Quartic(&curve,
			*pos,
			pState->toPos,
			*vel,
			pState->toVel,
			time_step,
			max_accel);
		
		pState->time_remaining = fclamp(maxaccel_time,0,max_time);
	}
	else
	{
		pState->time_remaining -= time_step;
	}
	
	if ( pState->time_remaining <= time_step )
	{
		*pos = pState->toPos;
		*vel = pState->toVel;
	}
	else
	{
		// solve cubic
		Bezier curve;
		
		const float T = (float) pState->time_remaining;
		
		curve.Fit(*pos,pState->toPos,
			*vel,pState->toVel,
			T);
		
		// step along by dt :
		float t = time_step / T;
		
		*pos = curve.X(t);
		*vel = curve.dX(t) * (1.f / T);
	}
}

}; //--------------------------------- namespace

#endif // SMOOTHDRIVER_H