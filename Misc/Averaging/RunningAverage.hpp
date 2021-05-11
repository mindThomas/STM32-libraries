/* Copyright (C) 2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#pragma once

#include <stddef.h>
#include <stdlib.h>

class RunningAverage
{
public:
    RunningAverage()
    {
        _nSamples = 0;
        _value    = 0;
    }

    ~RunningAverage() {}

    float Filter(float input)
    {
        float prev_sum = _value * _nSamples;
        float new_sum  = prev_sum + input;
        _nSamples++;
        _value = new_sum / _nSamples;
    }

private:
    uint32_t _nSamples; // number of currently averaged samples
    float    _value;
};