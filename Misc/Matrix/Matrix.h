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
 
#ifndef MISC_MATRIX_H
#define MISC_MATRIX_H

#include <stddef.h>
#include <stdlib.h>

#include "inv6x6.h"
#include "inv3x3.h"

class Matrix
{

public:
	Matrix();
	~Matrix();

private:
	
};
	
	
extern void Matrix_Extract(const float * in, const int in_rows, const int in_cols, const int in_row, const int in_col, const int out_rows, const int out_cols, float * out);
extern void Matrix_Round(float * matrix, int rows, int cols);
extern void Matrix_Print(float * matrix, int rows, int cols);
	
	
#endif
