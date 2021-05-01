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

#include "FirstOrderLPF.hpp"
#include <MathLib/MathLib.h>
#include <cmath>

FirstOrderLPF::FirstOrderLPF(float Ts, float freq3dB)
    : _Ts(Ts)
{
    ChangeFrequency(freq3dB);
    Reset();
}

FirstOrderLPF::~FirstOrderLPF() {}

// Filter a given input using the first order LPF
float FirstOrderLPF::Filter(float input)
{
    float out = _coeff_b * input + _coeff_b * _inputOld - _coeff_a * _lpfOld; // IIR difference equation implementation
    _lpfOld   = out;
    _inputOld = input;
    return out;
}

void FirstOrderLPF::Reset(void)
{
    _inputOld = 0;
    _lpfOld   = 0;
}

void FirstOrderLPF::ChangeFrequency(float freq3dB)
{
    // Calculate filter coefficients for a  of First order Low-pass filter using the Tustin (Bilinear) transform with
    // frequency warping
    float omega_continuous        = M_2PI * freq3dB;
    float omega_digital           = _Ts * omega_continuous; // omega_continuous / fs
    float omega_continuous_warped = 2.f / _Ts * tanf(omega_digital / 2.f);
    float tau                     = 1.f / omega_continuous_warped;

    _coeff_b = 1.f / (2.f * tau / _Ts + 1.f);                           // nominator
    _coeff_a = 1.f / (2.f * tau / _Ts + 1.f) - 2.f / (2.f + _Ts / tau); // denominator
    _freq3dB = freq3dB;
}
