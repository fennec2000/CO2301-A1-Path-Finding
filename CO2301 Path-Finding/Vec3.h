#ifndef __CVEC3
#define __CVEC3

#pragma once
#include <cmath>

using namespace std;

/*
Notes:
This is a collection of useful functions and class.
Somethings have changed but the majority of this is given from lessons at UClan.
Thanks to Gareth Bellaby and Nick Mitchell for source of this code.
*/

class Vec3
{
private:
public:
	// Vector components
	float x;
	float y;
	float z;

	// functions
	// constructors
	Vec3();
	Vec3(float xInput, float yInput, float zInput);
	Vec3(float Input[3]);

	// other
	float Dot(const Vec3& v);
};

const float kfEpsilon = 0.5e-6f;    // For 32-bit floats
const Vec3 kYAxis(0.0f, 1.0f, 0.0f);

inline Vec3 Subtract(const Vec3& v, const Vec3& w);
inline Vec3 Cross(const Vec3& v1, const Vec3& v2);
inline bool IsZero(const float x);
inline float InvSqrt(const float x);
Vec3 Normalise(const Vec3& v);

#endif