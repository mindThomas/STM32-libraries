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

#include "Servo.hpp"
#include <math.h>

Servo::Servo(timer_t timer, pwm_channel_t channel, float min, float max, float min_ms, float max_ms,
             uint16_t range_resolution_steps)
    : PWM(timer, channel, 50, (uint16_t)roundf(20.0f / ((max_ms - min_ms) / range_resolution_steps)))
    , // 50 Hz = 20 ms with '_ms_resolution' resolution
    _min_ms(min_ms)
    , _min_value(min)
    , _max_ms(max_ms)
    , _max_value(max)
{
    uint16_t PWM_Max = (uint16_t)roundf(20.0f / ((max_ms - min_ms) / range_resolution_steps));
    _ms_resolution   = 1000.0f / (50.0f * PWM_Max);
    //_ms_resolution = (max_ms - min_ms) / range_resolution_steps;
    Disable();
}

void Servo::Set(float value)
{
    if (value < _min_value)
        value = _min_value;
    if (value > _max_value)
        value = _max_value;

    // Perform linear interpolation between min and max to compute ms output
    float ms_diff = _max_ms - _min_ms;
    float ms      = ms_diff * (value - _min_value) / (_max_value - _min_value) + _min_ms;

    // Convert output ms into PWM value
    SetRaw((uint16_t)roundf(ms / _ms_resolution));
}

void Servo::Disable()
{
    SetRaw(0);
}
