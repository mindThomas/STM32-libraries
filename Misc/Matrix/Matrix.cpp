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

#include "Matrix.hpp"
#include "Matrix.h"

#include <algorithm> // std::swap etc.
#include <arm_math.h>
#include <malloc.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef MATRIX_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) ((void)0U); // not implemented
#endif

/* Matrices are stored in memory in Row-major format
 * This means that elements next to each other in memory corresponds to elements next to each other of the same row
 * Assuming MATLAB syntax: mat(i,j)   where i denotes a row index and j and column index of a matrix of size m x n   (m
 * rows, n columns) Then the corresponding memory entry is given as: mat[n*i + j]
 */

// Matrix constructor
Matrix::Matrix(uint8_t num_rows, uint8_t num_cols)
    : rows_{num_rows}
    , cols_{num_cols}
    , size_{(uint16_t)num_rows * (uint16_t)num_cols}
{
    mat = (float*)malloc((uint32_t)num_rows * (uint32_t)num_cols * sizeof(float));
    if (mat) {
        free_at_destruction = true;
        // memset(mat, 0, (uint32_t)size_*sizeof(float)); // removed for speed
        arm_mat_init_f32(&mat_, rows_, cols_, mat);
    }
}

// Vector constructor
Matrix::Matrix(uint8_t num_elements)
    : rows_{num_elements}
    , cols_{1}
    , size_{num_elements}
{
    mat = (float*)malloc((uint32_t)num_elements * sizeof(float));
    if (mat) {
        free_at_destruction = true;
        // memset(mat, 0, (uint32_t)size_*sizeof(float)); // removed for speed
        arm_mat_init_f32(&mat_, rows_, cols_, mat);
    }
}

// Copy operator
Matrix::Matrix(const Matrix& other)
    : rows_{other.rows_}
    , cols_{other.cols_}
    , size_{(uint16_t)other.rows_ * (uint16_t)other.cols_}
{
    mat = (float*)malloc((uint32_t)other.size_ * sizeof(float));
    if (mat) {
        free_at_destruction = true;
        arm_mat_init_f32(&mat_, rows_, cols_, mat);
        memcpy(mat, other.mat, (uint32_t)size_ * sizeof(float));
    }
}

Matrix::Matrix(uint8_t num_rows, uint8_t num_cols, const float* move_mat)
    : rows_{num_rows}
    , cols_{num_cols}
    , size_{(uint16_t)num_rows * (uint16_t)num_cols}
    , mat{(float*)move_mat}
    , free_at_destruction{false}
{
    arm_mat_init_f32(&mat_, rows_, cols_, mat);
}

Matrix::~Matrix()
{
    if (mat && free_at_destruction) {
        free(mat);
    }
}

float& Matrix::operator()(uint8_t row)
{
    if (row >= rows_ || cols_ == 0) {
        tmp = 0;
        return tmp;
    }

    // Row-major order
    return mat[row * cols_];
}

const float& Matrix::operator()(uint8_t row) const
{
    if (row >= rows_ || cols_ == 0) {
        return tmp;
    }

    // Row-major order
    return mat[row * cols_];
}

float& Matrix::operator()(uint8_t row, uint8_t col)
{
    if (row >= rows_ || col >= cols_) {
        tmp = 0;
        return tmp;
    }

    // Row-major order
    return mat[row * cols_ + col];
}

const float& Matrix::operator()(uint8_t row, uint8_t col) const
{
    if (row >= rows_ || col >= cols_) {
        return tmp;
    }

    // Row-major order
    return mat[row * cols_ + col];
}

// Assignment operator
Matrix& Matrix::operator=(const Matrix& other)
{
    if (this != &other) { // self-assignment check expected
                          //~Matrix();  // call own destructor
        if (mat) {
            free(mat);
        }
        // Copy operation
        rows_ = other.rows_;
        cols_ = other.cols_;
        size_ = other.size_;
        mat   = (float*)malloc((uint32_t)size_ * sizeof(float));
        if (mat) {
            arm_mat_init_f32(&mat_, rows_, cols_, mat);
            memcpy(mat, other.mat, (uint32_t)size_ * sizeof(float));
        }
    }
    return *this;
}

// Move assignment
Matrix& Matrix::operator=(Matrix&& other) noexcept
{
    if (this != &other) { // no-op on self-move-assignment (delete[]/size=0 also ok)
                          //~Matrix();  // call own destructor
        if (mat) {
            free(mat);
        }
        mat       = other.mat;
        other.mat = 0;
        rows_     = other.rows_;
        cols_     = other.cols_;
        size_     = other.size_;
        arm_mat_init_f32(&mat_, rows_, cols_, mat);
    }
    return *this;
}

/*
// copy/move constructor is called to construct arg
Matrix& Matrix::operator=(Matrix& other) noexcept
{
    // resources are exchanged between *this and arg
    //~Matrix();  // call own destructor
    if (mat) {
        free(mat);
    }
    new (this) Matrix(other); // reconstruct object

    return *this;
} // destructor of arg is called to release the resources formerly held by *this
*/

// Type-casting to arm_matrix_instance_f32 pointer
Matrix::operator arm_matrix_instance_f32*()
{
    return &mat_;
}

Matrix::operator arm_matrix_instance_f32*() const
{
    return (arm_matrix_instance_f32*)&mat_;
}

// Type-casting to arm_matrix_instance_f32 pointer
Matrix::operator float32_t*()
{
    return mat;
}

// Type-casting to arm_matrix_instance_f32 pointer
Matrix::operator float32_t*() const
{
    return (float32_t*)mat;
}

void Matrix::zero()
{
    memset(mat, 0, (uint32_t)size_ * sizeof(float));
}

// Inline transpose
void Matrix::transpose()
{
    // Needs to copy and apply transformation
    Matrix* transformed = new Matrix(rows_, cols_);
    arm_mat_trans_f32(*this, *transformed);

    if (mat) {
        free(mat);
    }
    mat              = transformed->mat;
    transformed->mat = 0;

    delete transformed;
}

void Matrix::transpose(Matrix& out) const
{
    // out = M^T
    if (rows_ == out.cols_ || cols_ == out.rows_)
        return; // error - sizes do not match

    arm_mat_trans_f32(*this, out);
}

Matrix Matrix::T() const
{
    // Needs to copy and apply transformation
    Matrix transformed(cols_, rows_);
    arm_mat_trans_f32(*this, transformed);

    return transformed;
}

Matrix Matrix::operator*(const Matrix& other) const
{
    // out = this * other
    Matrix out(rows_, other.cols_);
    multiply(*this, other, out);
    return out;
}

Matrix Matrix::inv() const
{
    // out = inv(this)
    // only for invertible [read: square] matrices!
    Matrix out(rows_, cols_);
    inverse(*this, out);
    return out;
}

float Matrix::det() const
{
    // out = det(this)
    return determinant(*this);
}

Matrix Matrix::diag() const
{
    if (cols_ == 1) {
        // Make diagonal matrix from vector
        Matrix out = Matrix::zeros(rows_, rows_);
        for (uint8_t i = 0; i < rows_; i++) {
            out(i, i) = (*this)(i);
        }
        return out;
    } else {
        // Make vector from the diagonal elements of a matrix
        uint8_t num_elements = rows_ < cols_ ? rows_ : cols_;
        Matrix  out          = Matrix::zeros(num_elements);
        for (uint8_t i = 0; i < num_elements; i++) {
            out(i) = (*this)(i, i);
        }
        return out;
    }
}

#ifdef MATRIX_USE_DEBUG
void Matrix::print(const char* preText) const
{
    if (preText) {
        Debug::print(preText);
        Debug::print(" = \n");
    }
    for (uint8_t m = 0; m < rows_; m++) {
        Debug::print(" ");
        for (uint8_t n = 0; n < cols_; n++) {
            Debug::printf("%8.4f ", mat[cols_ * m + n]);
        }
        Debug::print("\n");
    }
}
#endif

/* Static functions - also kept for legacy support reasons */

void Matrix_Extract(const float* in, const int in_rows, const int in_cols, const int in_row, const int in_col,
                    const int out_rows, const int out_cols, float* out)
{
    /*assert(out_rows <= in_rows);
    assert(out_cols <= in_cols);
    assert((in_row + out_rows) < in_rows);
    assert((in_col + out_cols) < in_cols);*/
    for (int m = 0; m < out_rows; m++) {
        for (int n = 0; n < out_cols; n++) {
            out[m * out_cols + n] = in[(in_row + m) * in_cols + (in_col + n)];
        }
    }
}

void Matrix_Round(float* matrix, int rows, int cols)
{
    for (int m = 0; m < rows; m++) {
        for (int n = 0; n < cols; n++) {
            matrix[cols * m + n] = roundf(matrix[cols * m + n] * 10) / 10;
            if (matrix[cols * m + n] == -0.f) {
                matrix[cols * m + n] = 0.f;
            }
        }
    }
}

#ifdef MATRIX_USE_DEBUG
void Matrix_Print(float* matrix, int rows, int cols)
{
    for (int m = 0; m < rows; m++) {
        Debug::print(" ");
        for (int n = 0; n < cols; n++) {
            Debug::printf("%8.4f ", matrix[cols * m + n]);
        }
        Debug::print("\n");
    }
}
#endif

arm_status Matrix_Multiply(const float* A, uint8_t A_rows, uint8_t A_cols, const float* B, uint8_t B_rows,
                           uint8_t B_cols, float* out, uint8_t out_rows, uint8_t out_cols)
{
    // See ARM-CMSIS DSP library for matrix operations:
    // https://www.keil.com/pack/doc/CMSIS/DSP/html/group__groupMatrix.html
    arm_matrix_instance_f32 A_;
    arm_mat_init_f32(&A_, A_rows, A_cols, (float32_t*)A);
    arm_matrix_instance_f32 B_;
    arm_mat_init_f32(&B_, B_rows, B_cols, (float32_t*)B);
    arm_matrix_instance_f32 out_;
    arm_mat_init_f32(&out_, out_rows, out_cols, out);
    return arm_mat_mult_f32(&A_, &B_, &out_);
}
