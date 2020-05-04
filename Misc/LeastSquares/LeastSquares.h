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
 
#ifndef MISC_LEAST_SQUARES_H
#define MISC_LEAST_SQUARES_H

#include <stddef.h>
#include <stdlib.h>

class LeastSquares
{

public:
	LeastSquares();
	~LeastSquares();

private:
	
};


/* Implementations of ordinary recursive least squares
 *  min ||a*x - y||^2
 *  where || is the 2-norm (Euclidean distance)
 *  with an recursively incoming value of a_n and measurement y_n
 */
class LeastSquares1D
{
	
};

class LeastSquares2D
{
	
};	

#endif
