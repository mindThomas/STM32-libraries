/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifndef MISC_RATELIMITER_H
#define MISC_RATELIMITER_H

#include "Debug.h"
#include <assert.h> // assert
#include <cmath>

class RateLimiter
{
public:
    RateLimiter(float Ts, float rateLimit)
        : _Ts(Ts)
        , _accelerationDeltaLimit(Ts * rateLimit)
        , _deccelerationDeltaLimit(Ts * rateLimit)
    {}

    RateLimiter(float Ts, float accelerationLimit, float deccelerationLimit)
        : _Ts(Ts)
        , _accelerationDeltaLimit(Ts * accelerationLimit)
        , _deccelerationDeltaLimit(Ts * deccelerationLimit)
    {}

    void reset(float prevOutput) { _prevOutput = prevOutput; }

    void set_rate_limit(float rateLimit)
    {
        _accelerationDeltaLimit  = _Ts * rateLimit;
        _deccelerationDeltaLimit = _Ts * rateLimit;
    }

    void set_rate_limits(float accelerationLimit, float deccelerationLimit)
    {
        _accelerationDeltaLimit  = _Ts * accelerationLimit;
        _deccelerationDeltaLimit = _Ts * deccelerationLimit;
    }

    float operator()(float input)
    {
        float delta = input - _prevOutput;
        if (fabsf(input) > fabsf(_prevOutput)) {
            _prevOutput += clamp(delta, -_accelerationDeltaLimit, _accelerationDeltaLimit);
        } else {
            _prevOutput += clamp(delta, -_deccelerationDeltaLimit, _deccelerationDeltaLimit);
        }
        return _prevOutput;
    }

private:
    template<class T>
    constexpr const T& clamp(const T& v, const T& lo, const T& hi)
    {
        assert(!(hi < lo));
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }

private:
    float _Ts;
    float _accelerationDeltaLimit;
    float _deccelerationDeltaLimit;
    float _prevOutput;
};

#endif
