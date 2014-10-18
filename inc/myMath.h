
#ifndef MYMATH_H
#define MYMATH_H

/* Original Math functions */

#include <Math.h>

#include "parameters.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define GRAVITY 9.767f

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define ToRad(x) (x * DEG_TO_RAD)
#define ToDeg(x) (x * RAD_TO_DEG)

float	Vector_Dot_Product(const Vector3f *v1, const Vector3f *v2);
void	Vector_Cross_Product(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Vector_Scale(Vector3f *out, const Vector3f *v, float scale);
void	Vector_Add(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Matrix_Multiply(const Matrix3f *m1, const Matrix3f *m2, Matrix3f *out);
void	Matrix_Vector_Multiply(const Matrix3f *m, const Vector3f *v, Vector3f *out);
void	Vector_Normalize(Vector3f *a);

#endif