#include "Matrix4x4.h"

void Matrix4x4::MakeIdentity()
{
	// not convectional way of writing but resembles the matrix closely
	e00 = 1.0f;		e01 = 0.0f;		e02 = 0.0f;		e03 = 0.0f;
	e10 = 0.0f;		e11 = 1.0f;		e12 = 0.0f;		e13 = 0.0f;
	e20 = 0.0f;		e21 = 0.0f;		e22 = 1.0f;		e23 = 0.0f;
	e30 = 0.0f;		e31 = 0.0f;		e32 = 0.0f;		e33 = 1.0f;
}

void Matrix4x4::SetRow(const int iRow, const Vec3& v)
{
	float* pfElts = &e00 + iRow * 4;
	pfElts[0] = v.x;
	pfElts[1] = v.y;
	pfElts[2] = v.z;
}