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
 
#include "Matrix.h"
#include <math.h>
#include <stdlib.h>
#include "Debug.h"
 
Matrix::Matrix()
{
	
}

Matrix::~Matrix()
{
	
}

/* Matrices are stored in memory in Row-major format
 * This means that elements next to each other in memory corresponds to elements next to each other of the same row
 * Assuming MATLAB syntax: mat(i,j)   where i denotes a row index and j and column index of a matrix of size m x n   (m rows, n columns)
 * Then the corresponding memory entry is given as: mat[n*i + j]
 */

void Matrix_Extract(const float * in, const int in_rows, const int in_cols, const int in_row, const int in_col, const int out_rows, const int out_cols, float * out)
{
	/*assert(out_rows <= in_rows);
	assert(out_cols <= in_cols);
	assert((in_row + out_rows) < in_rows);
	assert((in_col + out_cols) < in_cols);*/
	for (int m = 0; m < out_rows; m++)
	{
		for (int n = 0; n < out_cols; n++) {
			out[m*out_cols + n] = in[(in_row+m)*in_cols + (in_col+n)];
		}
	}
}

void Matrix_Round(float * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
	for (int n = 0; n < cols; n++) {
	  matrix[cols*m + n] = roundf(matrix[cols*m + n] * 10) / 10;
	  if (matrix[cols*m + n] == -0.f) {
		matrix[cols*m + n] = 0.f;
	  }
	}
  }
}

void Matrix_Print(float * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
	Debug::print(" ");
	for (int n = 0; n < cols; n++) {
		Debug::printf("%8.4f ", matrix[cols*m + n]);
	}
	Debug::print("\n");
  }
}
