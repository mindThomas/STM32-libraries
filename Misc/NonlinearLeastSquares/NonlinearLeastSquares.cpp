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

#include "NonlinearLeastSquares.hpp"
#include "Debug.h"
#include "Matrix.hpp"
#include <math.h>
#include <string.h>

void NonlinearLeastSquares::fitExponentialDecay(const Matrix& x, const Matrix& y, float coeffs_out[3])
{
    // Fit a function of the type: f(x) = a + b * exp(c*x)
    // - input: x, y
    // - output: coeffs = [a, b, c]
    // x and y should be column vectors; m x 1
    // Based on fitExponentialDecay2.m
    if (x.cols() != 1 || y.cols() != 1)
        return;
    if (x.rows() != y.rows())
        return;

    const float    x0 = x(0);
    const uint16_t n  = y.rows();

    float coeffs[3] = {0.f, y(0), -0.1f};

    float  Jr_error_[3] = {0, 0, 0};
    Matrix Jr_error(3, 1, Jr_error_);
    float  Jr_row[3];

    float  JrT_Jr_[9];
    Matrix JrT_Jr(3, 3, JrT_Jr_);

    float  invJrT_Jr_[9];
    Matrix invJrT_Jr(3, 3, invJrT_Jr_);

    float  delta_[3];
    Matrix delta(3, 1, delta_);

    // Perform Gauss Newton iteration
    while (1) { // iterate until convergence
        memset(Jr_error_, 0, 3 * sizeof(float));
        for (uint16_t i = 0; i < n; i++) {
            float x_  = x(i) - x0;
            float err = y(i) - model(x_, coeffs);
            JacobianRow(x_, coeffs, Jr_row);
            Jr_error_[0] += err * Jr_row[0];
            Jr_error_[1] += err * Jr_row[1];
            Jr_error_[2] += err * Jr_row[2];
        }

        memset(JrT_Jr_, 0, 9 * sizeof(float));
        for (uint16_t i = 0; i < n; i++) {
            JacobianRow(x(i) - x0, coeffs, Jr_row);
            JrT_Jr(0, 0) += Jr_row[0] * Jr_row[0];
            JrT_Jr(1, 0) += Jr_row[1] * Jr_row[0];
            JrT_Jr(2, 0) += Jr_row[2] * Jr_row[0];

            JrT_Jr(0, 1) += Jr_row[0] * Jr_row[1];
            JrT_Jr(1, 1) += Jr_row[1] * Jr_row[1];
            JrT_Jr(2, 1) += Jr_row[2] * Jr_row[1];

            JrT_Jr(0, 2) += Jr_row[0] * Jr_row[2];
            JrT_Jr(1, 2) += Jr_row[1] * Jr_row[2];
            JrT_Jr(2, 2) += Jr_row[2] * Jr_row[2];
        }

        Matrix::inverse(JrT_Jr, invJrT_Jr);
        Matrix::multiply(invJrT_Jr, Jr_error, delta);

        // Update coefficients
        coeffs[0] -= delta_[0];
        coeffs[1] -= delta_[1];
        coeffs[2] -= delta_[2];
        if (coeffs[2] > 0) {
            // reset c coefficient if it becomes positive, since that would result in NaN
            // given that this method and model is only valid for exponential decay (negative c coefficient)
            coeffs[2] = -0.1;
        }

        // Convergence criteria
        if (Matrix::norm(delta) < 0.01f * fabsf(coeffs[2]))
            break;
    }

    // Extract estimated parameters
    coeffs_out[0] = coeffs[0];                               // a
    coeffs_out[2] = coeffs[2];                               // c
    coeffs_out[1] = coeffs[1] * expf(coeffs_out[2] * (-x0)); // correct parameter with offset shift
}

float NonlinearLeastSquares::model(const float x, const float* coeffs)
{
    return coeffs[0] + coeffs[1] * expf(coeffs[2] * x); // model = a + b.*exp(c.*x)
}

void NonlinearLeastSquares::JacobianRow(const float x, const float* coeffs, float* Jr_row)
{
    Jr_row[0] = -1.0f;
    Jr_row[1] = -expf(coeffs[2] * x);
    Jr_row[2] = coeffs[1] * x * Jr_row[1];
}
