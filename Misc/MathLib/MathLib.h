/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef MISC_MATHLIB_H
#define MISC_MATHLIB_H

#include <stddef.h>
#include <stdlib.h>

/* Constants rounded for 21 decimals. */
#define M_E 2.71828182845904523536f
#define M_LOG2E 1.44269504088896340736f
#define M_LOG10E 0.434294481903251827651f
#define M_LN2 0.693147180559945309417f
#define M_LN10 2.30258509299404568402f
//#define M_PI 3.14159265358979323846264338327950288f
#define M_PI 3.14159265358979323846f
#define M_2PI 6.283185307179586476925286766559005f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.785398163397448309616f
#define M_1_PI 0.318309886183790671538f
#define M_2_PI 0.636619772367581343076f
#define M_1_SQRTPI 0.564189583547756286948f
#define M_2_SQRTPI 1.12837916709551257390f
#define M_SQRT2 1.41421356237309504880f
#define M_SQRT_2 0.707106781186547524401f

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

// Example number = 37.777779
#define ROUND_DOWN_2DEC(x) (floorf(x * 100) / 100)    /* Result: 37.77 */
#define ROUND_2DEC(x) 	   (roundf(x * 100) / 100)    /* Result: 37.78 */
#define ROUND_UP_2DEC(x)   (ceilf(x * 100) / 100)     /* Result: 37.78 */

float Math_Round(float num, unsigned int dec);
void Math_SymmetrizeSquareMatrix(float * mat, unsigned int rows);
void Math_Rotate2D(const float V[2], const float theta, float Vr[2]);

#endif
