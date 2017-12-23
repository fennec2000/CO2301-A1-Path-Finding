#include "Vec3.h"

Vec3::Vec3()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

Vec3::Vec3(float xInput, float yInput, float zInput)
{
	x = xInput;
	y = yInput;
	z = zInput;
}

Vec3::Vec3(float Input[3])
{
	x = Input[0];
	y = Input[1];
	z = Input[2];
}

// using & to get the vector without copying and using const so it cant be changed by accident
// inline is a small optimisation for the compiler
// return the dot of this vector with the given vector
float Vec3::Dot(const Vec3& v)
{
	return x*v.x + y*v.y + z*v.z;
}

// Subtracting two given vectors (order is important - non-member version
Vec3 Subtract(const Vec3& v, const Vec3& w)
{
	return Vec3(v.x - w.x, v.y - w.y, v.z - w.z);
}

// Dot product of two given vectors (order not important) - non-member version
Vec3 Cross(const Vec3& v1, const Vec3& v2)
{
	return Vec3(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}

// Test if a float value is approximately 0. Epsilon value is the range around zero that
// is considered equal to zero. Default value requires zero to 6 decimal places
bool IsZero(const float x)
{
	return fabsf(x) < kfEpsilon;
}

// 1 / Sqrt
float InvSqrt(const float x)
{
	return 1.0f / sqrtf(x);
}

// Return unit length vector in the same direction as given one
Vec3 Normalise(const Vec3& v)
{
	float lengthSq = v.x*v.x + v.y*v.y + v.z*v.z;

	// Ensure vector is not zero length (use BaseMath.h float approx. fn with default epsilon)
	if (IsZero(lengthSq))
	{
		return Vec3(0.0f, 0.0f, 0.0f);
	}
	else
	{
		float invLength = InvSqrt(lengthSq);
		return Vec3(v.x * invLength, v.y * invLength, v.z * invLength);
	}
}