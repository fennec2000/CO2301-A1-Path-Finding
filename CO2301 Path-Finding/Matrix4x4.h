#ifndef __MATRIX4X4
#define __MATRIX4X4

#pragma once
#include "Vec3.h"

/*
Notes:
This is a collection of useful functions and class.
Somethings have changed but the majority of this is given from lessons at UClan.
Thanks to Gareth Bellaby and Nick Mitchell for source of this code.
*/

class Matrix4x4
{
	// Concrete class - public access
public:
	// Matrix elements
	float e00, e01, e02, e03;
	float e10, e11, e12, e13;
	float e20, e21, e22, e23;
	float e30, e31, e32, e33;

	// Default constructor - leaves values uninitialised (for performance)
	Matrix4x4() {}

	// Make this matrix the identity matrix
	void MakeIdentity();

	// Set a single row (range 0-3) of the matrix using a Vec3. Fourth element left unchanged
	void Matrix4x4::SetRow(const int iRow, const Vec3& v);
};

#endif