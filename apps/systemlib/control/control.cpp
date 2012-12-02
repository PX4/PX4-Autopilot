/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file control.cpp
 *
 * Controller library code
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "control.h"

namespace control
{

// Limit methods

Limit::Limit() :
    _min(0), _max(0)
{
}

float Limit::update(float input, uint16_t dt)
{
    if (input > getMax())
    {
        input = getMax();
    }
    else if (input < getMin())
    {
        input = getMin();
    }
    return input;
}

Limit::~Limit()
{
}

// LowPass methods

LowPass::LowPass() :
    _state(0), _fCut(0)
{
}

float LowPass::update(float input, uint16_t dt)
{
    float b = 2*M_PI*getFCut()*dt;
    float a = b/ (1 + b);
    setState(a*input + (1-a)*getState());
    return getState();
}

LowPass::~LowPass()
{
}

// Integral methods

Integral::Integral() :
    _state(0), _limit()
{
}

float Integral::update(float input, uint16_t dt)
{
    // trapezoidal integration
    setState(_limit.update(getState() + 
                (getState() + input)*dt/2,dt));
    return getState();
}

Integral::~Integral()
{
}

// Derivative methods

Derivative::Derivative() :
    _state(0)
{
}

float Derivative::update(float input, uint16_t dt)
{
    float output = (input - getState())/dt;
    setState(input);
    return output;
}

Derivative::~Derivative()
{
}

// PID methods

PID::PID() :
    _kP(0), _kI(0), _kD(0), _integral(), _derivative()
{
}

float PID::update(float input, uint16_t dt)
{
    return getKP()*input + 
        getKI()*getIntegral().update(input,dt) +
        getKD()*getDerivative().update(input,dt);
}

PID::~PID()
{
}


} // namespace control
