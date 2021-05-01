/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include <arm_math.h>
#include <stddef.h>
#include <stdlib.h>

#include "inv2x2.h"
#include "inv3x3.h"
#include "inv4x4.h"
#include "inv5x5.h"
#include "inv6x6.h"

#include "det2x2.h"
#include "det3x3.h"
#include "det4x4.h"
#include "det5x5.h"
#include "det6x6.h"

#include "svd_2x2.h"
#include "svd_3x3.h"
#include "svd_4x4.h"
#include "svd_5x5.h"
#include "svd_6x6.h"

class Matrix
{

private:
    uint8_t  rows_{0};
    uint8_t  cols_{0};
    uint16_t size_{0};

    float*                  mat{0};
    bool                    free_at_destruction{false};
    arm_matrix_instance_f32 mat_;
    float                   tmp; // used for invalid assignments

public:
    Matrix();
    Matrix(const Matrix& other);
    Matrix(uint8_t num_elements);               // vector constructor
    Matrix(uint8_t num_rows, uint8_t num_cols); // matrix constructor
    Matrix(uint8_t num_rows, uint8_t num_cols, const float* move_mat);
    ~Matrix();

    inline uint8_t  rows() const { return rows_; };
    inline uint8_t  cols() const { return cols_; };
    inline uint16_t size() const { return size_; };

    float&       operator()(uint8_t row);
    const float& operator()(uint8_t row) const;
    float&       operator()(uint8_t row, uint8_t col);
    const float& operator()(uint8_t row, uint8_t col) const;
    Matrix&      operator=(const Matrix& other);
    Matrix&      operator=(Matrix&& other) noexcept;
    // Matrix& operator=(Matrix other) noexcept;

    Matrix operator*(const Matrix& other) const;

    void zero();

    void   transpose();
    void   transpose(Matrix& out) const;
    Matrix T() const;
    Matrix inv() const;
    float  det() const;
    Matrix diag() const;

    void print(const char* preText = 0) const;

private:
    operator arm_matrix_instance_f32*();
    operator arm_matrix_instance_f32*() const;
    operator float32_t*();
    operator float32_t*() const;

    // Static and inline functions
public:
    static Matrix zeros(uint8_t num_elements)
    { // zero-vector constructor
        Matrix out(num_elements);
        out.zero();
        return out;
    }

    static Matrix zeros(uint8_t num_rows, uint8_t num_cols)
    { // matrix constructor
        Matrix out(num_rows, num_cols);
        out.zero();
        return out;
    }

    static inline void multiply(const Matrix& A, const Matrix& B, Matrix& out)
    {
        // out = A * B
        if (out.rows_ != A.rows_ || out.cols_ != B.cols_ || A.cols_ != B.rows_)
            return; // error - sizes do not match

        arm_mat_mult_f32(A, B, out);
    }

    static inline void add(const Matrix& A, const Matrix& B, Matrix& out)
    {
        // out = A + B
        if (out.rows_ == A.rows_ || out.cols_ == A.cols_ || A.rows_ != B.rows_ || A.cols_ != B.cols_)
            return; // error - sizes do not match

        // arm_mat_add_f32(A, B, out);
        arm_add_f32(A, B, out, out.size_);
    }

    static inline void subtract(const Matrix& A, const Matrix& B, Matrix& out)
    {
        // out = A - B
        if (out.rows_ == A.rows_ || out.cols_ == A.cols_ || A.rows_ != B.rows_ || A.cols_ != B.cols_)
            return; // error - sizes do not match

        // arm_mat_sub_f32(A, B, out);
        arm_sub_f32(A, B, out, out.size_);
    }

    static inline void transpose(const Matrix& M, Matrix& out)
    {
        // out = M^T
        if (M.rows_ == out.rows_ || M.cols_ == out.cols_)
            return; // error - sizes do not match

        arm_mat_trans_f32(M, out);
    }

    static inline void inverse(const Matrix& M, Matrix& out)
    {
        // Only possible for square matrices
        if (M.rows_ != M.cols_)
            return;
        if (out.rows_ != M.rows_ || out.cols_ != M.cols_)
            return;

        if (M.rows_ == 2) {
            inv2x2_fast(M.mat, out.mat);
        } else if (M.rows_ == 3) {
            inv3x3(M.mat, out.mat);
        } else if (M.rows_ == 4) {
            inv4x4(M.mat, out.mat);
        } else if (M.rows_ == 5) {
            inv5x5(M.mat, out.mat);
        } else if (M.rows_ == 6) {
            inv6x6(M.mat, out.mat);
        }
    }

    static inline float determinant(const Matrix& M)
    {
        // Only possible for square matrices
        if (M.rows_ != M.cols_)
            return 0;

        if (M.rows_ == 2) {
            return det2x2_fast(M.mat);
        } else if (M.rows_ == 3) {
            return det3x3(M.mat);
        } else if (M.rows_ == 4) {
            return det4x4(M.mat);
        } else if (M.rows_ == 5) {
            return det5x5(M.mat);
        } else if (M.rows_ == 6) {
            return det6x6(M.mat);
        }
    }

    static inline void svd(const Matrix& M, Matrix& U, Matrix& S, Matrix& V)
    {
        // Currently only implemented for square matrices
        if (M.rows_ != M.cols_)
            return;
        if (U.rows_ != M.rows_ || U.cols_ != M.cols_)
            return;
        if (S.rows_ != M.rows_ || S.cols_ != M.cols_)
            return;
        if (V.rows_ != M.rows_ || V.cols_ != M.cols_)
            return;

        if (M.rows_ == 2) {
            svd_2x2(M.mat, U.mat, S.mat, V.mat);
        } else if (M.rows_ == 3) {
            svd_3x3(M.mat, U.mat, S.mat, V.mat);
        } else if (M.rows_ == 4) {
            svd_4x4(M.mat, U.mat, S.mat, V.mat);
        } else if (M.rows_ == 5) {
            svd_5x5(M.mat, U.mat, S.mat, V.mat);
        } else if (M.rows_ == 6) {
            svd_6x6(M.mat, U.mat, S.mat, V.mat);
        }
    }

    static inline float norm(const Matrix& V)
    {
        // Currently only implemented for vectors (Euclidean vector norm)
        if (V.cols_ != 1)
            return 0;

        float res = 0;
        for (uint8_t i = 0; i < V.rows_; i++) {
            res += V.mat[i] * V.mat[i]; // use fast lookup since we know this is just a vector
        }

        return sqrtf(res);
    }

    inline void negate() { arm_negate_f32(*this, *this, this->size_); }
};

extern void Matrix_Extract(const float* in, const int in_rows, const int in_cols, const int in_row, const int in_col,
                           const int out_rows, const int out_cols, float* out);
extern void Matrix_Round(float* matrix, int rows, int cols);
extern void Matrix_Print(float* matrix, int rows, int cols);

#endif
