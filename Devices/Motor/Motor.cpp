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

#include "Motor.h"

Motor::Motor()
    : _motorDriver(0)
    , _currentSense(0)
    , _encoder(0)
{}

// Set output shaft (after gearing) torque in Newton meters (Nm)
// Returns a boolean indicating whether the applied torque was saturated/clipped
bool Motor::SetOutputTorque(float torqueNewtonMeter)
{
    if (!_motorDriver)
        return false;
    return _motorDriver->SetTorque(torqueNewtonMeter / GEARING_RATIO);
}

// Return applied torque (based on current reading) on the output shaft (after gearing) in Newton meters (Nm)
float Motor::GetAppliedOutputTorque()
{
    if (!_motorDriver)
        return 0;
    return GEARING_RATIO * _motorDriver->GetAppliedTorque();
}

int32_t Motor::GetEncoderRaw()
{
    if (!_encoder)
        return 0;
    return _encoder->Get();
}

// Return motor angle in radians (rad)
float Motor::GetAngle()
{
    if (!_encoder)
        return 0;
    int32_t encoderReading = _encoder->Get();

    // The encoder reading is in number of quadrature ticks (counting each edge on the two signal wires, hence 4 ticks
    // pr. repetition)
    float absoluteMotorRevolutions = (float)encoderReading / ENCODER_TICKS_PR_REV;

    // Since the motor is geared the absolute output shaft angle is less than the absolute motor shaft angle
    float absoluteOutputRevolutions = absoluteMotorRevolutions / GEARING_RATIO;

    // Convert to radians
    return 2 * M_PI * absoluteOutputRevolutions;
}
