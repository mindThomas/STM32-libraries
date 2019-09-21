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

#define M_PI 3.14159265358979323846264338327950288f

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
