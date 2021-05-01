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

#include "QuadratureKnob.h"
#include "IO.h"

QuadratureKnob::QuadratureKnob(GPIO_TypeDef* GPIOx_A, uint32_t GPIO_Pin_A, GPIO_TypeDef* GPIOx_B, uint32_t GPIO_Pin_B)
    : value(0)
    , oldAB(0)
{
    sigA = new IO(GPIOx_A, GPIO_Pin_A, IO::PULL_NONE);
    sigB = new IO(GPIOx_B, GPIO_Pin_B, IO::PULL_NONE);

    if (!sigA || !sigB) {
        if (sigA)
            delete (sigA);
        if (sigB)
            delete (sigB);
    } else {
        sigA->RegisterInterrupt(IO::TRIGGER_BOTH, QuadratureKnob::InterruptHandler, (void*)this);
        sigB->RegisterInterrupt(IO::TRIGGER_BOTH, QuadratureKnob::InterruptHandler, (void*)this);
    }
}

QuadratureKnob::~QuadratureKnob()
{
    if (sigA)
        delete (sigA);

    if (sigB)
        delete (sigB);
}

int32_t QuadratureKnob::Get()
{
    return value;
}

// Quadrature handling using Gray-code encoding scheme
// See image: https://www.allaboutcircuits.com/uploads/articles/rotary-encoder-waveform-v2.jpg
const int8_t QuadratureKnob::EncStates[16] = {
  // Encoder lookup table when triggering on both edges
  0,  // 00 -> 00
  -1, // 00 -> 01    (counter-clockwise)
  1,  // 00 -> 10    (clockwise)
  0,  // 00 -> 11
  1,  // 01 -> 00    (clockwise)
  0,  // 01 -> 01
  0,  // 01 -> 10
  -1, // 01 -> 11    (counter-clockwise)
  -1, // 10 -> 00    (counter-clockwise)
  0,  // 10 -> 01
  0,  // 10 -> 10
  1,  // 10 -> 11    (clockwise)
  0,  // 11 -> 00
  1,  // 11 -> 01    (clockwise)
  -1, // 11 -> 10    (counter-clockwise)
  0   // 11 -> 11
};
void QuadratureKnob::InterruptHandler(void* params)
{
    QuadratureKnob* QK = (QuadratureKnob*)params;

    // Shift previous state - this will insert a 0 at bottom bit
    QK->oldAB <<= 1;
    // Update what becomes bit 1 with current/new state
    QK->oldAB |= QK->sigA->Read();

    // Shift previous state - this will insert a 0 at bottom bit
    QK->oldAB <<= 1;
    // Update what becomes bit 0 with current/new state
    QK->oldAB |= QK->sigB->Read();

    // Use Gray-code encoding/decoding scheme to get value increment
    // The bottom 4 bits of oldAB now contains:
    // _oldAB & 0x0F   =   prevA, prevB, newA, newB
    QK->value += EncStates[QK->oldAB & 0x0F];
}
