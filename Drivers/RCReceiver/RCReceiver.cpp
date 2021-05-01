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

#include "RCReceiver.hpp"
#include <math.h>

RCReceiver::RCReceiver(InputCapture::timer_t timer, InputCapture::ic_channel_t channel, float min_ms, float max_ms)
    : InputCapture(timer, channel, 0.1f)
    , _min(min_ms / 1000)
    , _max(max_ms / 1000)
    , _prev_value(0)
{}

bool RCReceiver::VerifyPeriod(void)
{
    return (fabs(GetPeriodTime() - RECEIVER_PERIOD) < RECEIVER_PERIOD_TOLERANCE);
}

bool RCReceiver::isActive(void)
{
    return VerifyPeriod();
}

float RCReceiver::Get(bool ClearAfterReading)
{
    if (!VerifyPeriod())
        return _prev_value;

    float high_time = GetHighTime();
    if (high_time == 0)
        return 0;
    if (high_time < _min)
        high_time = _min;
    if (high_time > _max)
        high_time = _max;

    _prev_value = (2 * (high_time - _min) / (_max - _min)) - 1.0f;

    if (ClearAfterReading)
        Clear();

    return _prev_value;
}
