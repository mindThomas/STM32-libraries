/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "LeastSquares.h"
#include "Debug.h"
#include <math.h>
#include <stdlib.h>

LeastSquares1D::LeastSquares1D()
{
    a_squared = 0;
    ay        = 0;
}

LeastSquares1D::LeastSquares1D(float a_init, float y_init)
{
    a_squared = a_init * a_init;
    ay        = a_init * y_init;
}

void LeastSquares1D::AddMeasurement(float a_n, float y_n)
{
    // a_n should be the new value of 'a' to append
    // http://pfister.ee.duke.edu/courses/ece586/ex_proj_2008.pdf
    // Use normal linear algebra to extend A'*A and A'*y
    a_squared += a_n * a_n;
    ay += a_n * y_n;
}

float LeastSquares1D::GetEstimate()
{
    // Check for singularity (not sufficient samples added)
    if (fabs(a_squared) < 0.01)
        return 0;

    // Pseudo inverse is given as
    // Ainv = inv(A'*A) * A';
    // Compute estimate
    // x = Ainv * y;
    float x_est = ay / a_squared;

    return x_est;
}

float LeastSquares1D::AddMeasurementAndGetEstimate(float a_n, float y_n)
{
    AddMeasurement(a_n, y_n);
    return GetEstimate();
}

/*********************************************************************************************************************/

LeastSquares2D::LeastSquares2D()
    : initialized{false}
    , initialization_row{0}
{
    A_init_[0] = 0;
    A_init_[1] = 0;
    A_init_[2] = 0;
    A_init_[3] = 0;
    y_init_[0] = 0;
    y_init_[1] = 0;
}

LeastSquares2D::LeastSquares2D(const float A_init[4], const float y_init[2])
    : initialized{false}
    , initialization_row{0}
{
    Initialize(A_init, y_init);
}

void LeastSquares2D::Initialize(const float A_init[4], const float y_init[2])
{
    // Pseudo inverse is given as
    // Ainv = inv(A'*A) * A';
    // x = Ainv * y;

    // Compute A'*A
    AtA[0] = A_init[0] * A_init[0] + A_init[2] * A_init[2];
    AtA[1] = A_init[0] * A_init[1] + A_init[2] * A_init[3];
    AtA[2] = A_init[1] * A_init[0] + A_init[3] * A_init[2];
    AtA[3] = A_init[1] * A_init[1] + A_init[3] * A_init[3];

    // Compute A'*y
    // AtY = A_init' * y_init
    AtY[0] = A_init[0] * y_init[0] + A_init[2] * y_init[1];
    AtY[1] = A_init[1] * y_init[0] + A_init[3] * y_init[1];

    initialized = true;
}

void LeastSquares2D::InitializeRow(uint8_t row, const float A_n[2], float y_n)
{
    if (row >= 2)
        return;

    A_init_[2 * row]     = A_n[0];
    A_init_[2 * row + 1] = A_n[1];
    y_init_[row]         = y_n;
}

void LeastSquares2D::Reset()
{
    initialized        = false;
    initialization_row = 0;
    A_init_[0]         = 0;
    A_init_[1]         = 0;
    A_init_[2]         = 0;
    A_init_[3]         = 0;
    y_init_[0]         = 0;
    y_init_[1]         = 0;
}

void LeastSquares2D::Reset(const float A_init[4], const float y_init[2])
{
    initialized        = false;
    initialization_row = 0;
    Initialize(A_init, y_init);
}

void LeastSquares2D::AddMeasurement(const float A_n[2], float y_n)
{
    if (!initialized) {
        InitializeRow(initialization_row, A_n, y_n);
        if (initialization_row == 0) {
            initialization_row = 1;
        } else {
            Initialize(A_init_, y_init_);
        }
        return;
    }

    // A_n should be the new row of A to append
    // http://pfister.ee.duke.edu/courses/ece586/ex_proj_2008.pdf
    // Use normal linear algebra to extend A'*A and A'*y
    // A'*A += A_n'*A_n;
    AtA[0] = AtA[0] + A_n[0] * A_n[0];
    AtA[1] = AtA[1] + A_n[0] * A_n[1];
    AtA[2] = AtA[2] + A_n[1] * A_n[0];
    AtA[3] = AtA[3] + A_n[1] * A_n[1];

    // A'*y += y_n*A_n';
    AtY[0] = AtY[0] + y_n * A_n[0];
    AtY[1] = AtY[1] + y_n * A_n[1];
}

bool LeastSquares2D::GetEstimate(float x_est[2])
{
    if (!initialized)
        return false;

    // Check for singularity (not sufficient samples added)
    if (fabs(AtA[0] * AtA[3] - AtA[1] * AtA[2]) < 0.01)
        return false;

    // Compute the inverse of A'*A
    float det  = 1.f / (AtA[0] * AtA[3] - AtA[1] * AtA[2]);
    AtA_inv[0] = det * AtA[3];
    AtA_inv[1] = -det * AtA[1];
    AtA_inv[2] = -det * AtA[2];
    AtA_inv[3] = det * AtA[0];

    // Compute estimate
    // x_est = AtA_inv * AtY
    x_est[0] = AtA_inv[0] * AtY[0] + AtA_inv[1] * AtY[1];
    x_est[1] = AtA_inv[2] * AtY[0] + AtA_inv[3] * AtY[1];

    return true;
}

void LeastSquares2D::AddMeasurementAndGetEstimate(const float A_n[2], float y_n, float x_est[2])
{
    AddMeasurement(A_n, y_n);
    GetEstimate(x_est);
}

/*********************************************************************************************************************/

/* LineFitter<LeastSquares1D>
 *  Fit line model:
 *    y = a*x
 */
template<>
void LineFitter<LeastSquares1D>::AddMeasurement(float x, float y)
{
    lsq.AddMeasurement(x, y);
}

template<>
LineFitter<LeastSquares1D>::lineParameters_t LineFitter<LeastSquares1D>::GetEstimate()
{
    lineParameters_t params{false, 0, 0};

    float est = lsq.GetEstimate();
    if (est != 0) {
        params.valid  = true;
        params.slope  = est;
        params.offset = 0;
    }

    return params;
}

/* LineFitter<LeastSquares2D>
 *  Fit line model:
 *    y = a*x + b
 */
template<>
void LineFitter<LeastSquares2D>::AddMeasurement(float x, float y)
{
    float A[2] = {x, 1.0f};
    lsq.AddMeasurement(A, y);
}

template<>
LineFitter<LeastSquares2D>::lineParameters_t LineFitter<LeastSquares2D>::GetEstimate()
{
    lineParameters_t params{false, 0, 0};

    float est[2];
    if (lsq.GetEstimate(est) != 0) {
        params.valid  = true;
        params.slope  = est[0];
        params.offset = est[1];
    }

    return params;
}
