/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef MATRIX_H
#define MATRIX_H

#include <arm_math.h>

void Matrix_Extract(const float* in, const int in_rows, const int in_cols, const int in_row, const int in_col,
                           const int out_rows, const int out_cols, float* out);
void Matrix_Round(float* matrix, int rows, int cols);

#ifdef MATRIX_USE_DEBUG
void Matrix_Print(float* matrix, int rows, int cols);
#endif

arm_status Matrix_Multiply(const float* A, uint8_t A_rows, uint8_t A_cols, const float* B, uint8_t B_rows,
                           uint8_t B_cols, float* out, uint8_t out_rows, uint8_t out_cols);

#endif // MATRIX_H