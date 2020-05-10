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
 
#ifndef MISC_NONLINEAR_LEAST_SQUARES_H
#define MISC_NONLINEAR_LEAST_SQUARES_H

#include "Matrix.hpp"


class NonlinearLeastSquares
{

public:
	static void fitExponentialDecay(const Matrix& x, const Matrix& y, float coeffs_out[3]);

private:	
	static float model(const float x, const float * coeffs);
	static void JacobianRow(const float x, const float * coeffs, float * Jr_row);

};
	
	
#endif
