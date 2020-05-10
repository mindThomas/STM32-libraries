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
 
#ifndef MISC_LEAST_SQUARES_H
#define MISC_LEAST_SQUARES_H

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>


/* Implementations of ordinary recursive least squares
 *  min ||y - a*x||^2
 *  where || is the 2-norm (Euclidean distance)
 *  with a recursively incoming value of a_n and measurement y_n
 *
 * Based on the work in: https://github.com/mindThomas/MATLAB-tools/blob/master/LeastSquares/RecursiveLeastSquares_1D.m
 */
class LeastSquares1D
{
private:
    // Pseudo inverse is given as
    // Ainv = inv(A'*A) * A';
    // x = Ainv * y;
	float a_squared; // A'*A
	float ay; // A' * y
	
public:
	LeastSquares1D();
	LeastSquares1D(float a_init, float y_init);
	void AddMeasurement(float a_n, float y_n);
	float GetEstimate();
	float AddMeasurementAndGetEstimate(float a_n, float y_n);
};

/* Different implementations of ordinary recursive least squares
 *  min ||y - A*x||^2
 *  where || is the 2-norm (Euclidean distance)
 *  with a recursively incoming 2-element row of A_n and measurement y_n
 *
 * Based on the work in: https://github.com/mindThomas/MATLAB-tools/blob/master/LeastSquares/RecursiveLeastSquares_2D.m
 */
class LeastSquares2D
{
private:
	uint8_t initialization_row{0};
	bool initialized{false};
	float A_init_[4];
	float y_init_[2];

    // Pseudo inverse is given as
    // Ainv = inv(A'*A) * A';
    // x = Ainv * y;
	float AtA[4]; // A'*A
	float AtA_inv[4]; // inv(A'*A)
	float AtY[2]; // A' * y

	// Matrices are stored in row-major format, that is:
	// A[0] = A(1,1)
	// A[1] = A(1,2)
	// A[2] = A(2,1)
	// A[3] = A(2,2)

private:
	void Initialize(const float A_init[4], const float y_init[2]);
	void InitializeRow(uint8_t row, const float A_n[2], float y_n);
	
public:
	LeastSquares2D();
	LeastSquares2D(const float A_init[4], const float y_init[2]);
	void Reset();
	void Reset(const float A_init[4], const float y_init[2]);
	void AddMeasurement(const float A_n[2], float y_n);
	bool GetEstimate(float x_est[2]);
	void AddMeasurementAndGetEstimate(const float A_n[2], float y_n, float x_est[2]);
};

class LineFitting
{
public:
	typedef struct {
		bool valid;
		float slope;
		float offset;
	} lineParameters_t;

public:
	LineFitting() = default;
	virtual ~LineFitting() = default;
	virtual void AddMeasurement(float x, float y) = 0;
	virtual lineParameters_t GetEstimate() = 0;
};

template <class LSQ>
class LineFitter : public LineFitting
{
private:
	LSQ lsq;

public:
	LineFitter() = default;
	void AddMeasurement(float x, float y);
	lineParameters_t GetEstimate();
};

#endif
