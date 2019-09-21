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
 
#include "FirstOrderLPF.h"
 
FirstOrderLPF::FirstOrderLPF(float Ts, float tau) : _Ts(Ts), _tau(tau),
	// Calculate filter coefficients for a  of First order Low-pass filter using the Tustin (Bilinear) transform (however without frequency warping)
	_coeff_b( 1/(2*tau/Ts + 1) ), // nominator
	_coeff_a( 1/(2*tau/Ts + 1) - 2/(2 + Ts/tau) ) // denominator
{
	_inputOld = 0;
	_lpfOld = 0;
}

FirstOrderLPF::~FirstOrderLPF()
{
}

// Filter a given input using the first order LPF
float FirstOrderLPF::Filter(float input)
{	
	float out = _coeff_b * input + _coeff_b * _inputOld - _coeff_a * _lpfOld; // IIR difference equation implementation
	_lpfOld = out;
	_inputOld = input;
	return out;
}

void FirstOrderLPF::Reset(void)
{
	_inputOld = 0;
	_lpfOld = 0;
}

void FirstOrderLPF::ChangeTimeconstant(float tau)
{
	_tau = tau;
	_coeff_b = 1/(2*tau/_Ts + 1); // nominator
	_coeff_a = 1/(2*tau/_Ts + 1) - 2/(2 + _Ts/tau); // denominator
}
