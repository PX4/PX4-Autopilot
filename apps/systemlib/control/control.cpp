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

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "control.h"

namespace control
{

// Limit methods

__EXPORT Limit::Limit() :
    _min(0), _max(0)
{
    printf("ctor Limit\n");
}

__EXPORT float Limit::update(float input, uint16_t dt)
{
    printf("update Limit\n");
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

__EXPORT Limit::~Limit()
{
    printf("dtor Limit\n");
}

// LowPass methods

__EXPORT LowPass::LowPass() :
    _state(0), _cutFreq(0)
{
    printf("ctor LowPass\n");
}

__EXPORT float LowPass::update(float input, uint16_t dt)
{
    printf("update LowPass");
    float b = 2*M_PI*getCutFreq()*dt;
    float a = b/ (1 + b);
    setState(a*input + (1-a)*getState());
    return getState();
}

__EXPORT LowPass::~LowPass()
{
    printf("dtor LowPass\n");
}

// Integral methods

__EXPORT Integral::Integral() :
    _state(0), _limit()
{
    printf("ctor Integral\n");
}

__EXPORT float Integral::update(float input, uint16_t dt)
{
    printf("update Integral");
    // trapezoidal integration
    setState(_limit.update(getState() + 
                (getState() + input)*dt/2,dt));
    return getState();
}

__EXPORT Integral::~Integral()
{
    printf("dtor Integral\n");
}

// Derivative methods

__EXPORT Derivative::Derivative() :
    _state(0)
{
    printf("ctor Derivative\n");
}

__EXPORT float Derivative::update(float input, uint16_t dt)
{
    printf("update Derivative\n");
    float output = (input - getState())/dt;
    setState(input);
    return output;
}

__EXPORT Derivative::~Derivative()
{
    printf("dtor Derivative\n");
}

// PID methods

__EXPORT PID::PID() :
    _kP(0), _kI(0), _kD(0), _integral(), _derivative()
{
    printf("ctor PID\n");
}

__EXPORT float PID::update(float input, uint16_t dt)
{
    printf("update PID\n");
    return getKP()*input + 
        getKI()*getIntegral().update(input,dt) +
        getKD()*getDerivative().update(input,dt);
}

__EXPORT PID::~PID()
{
    printf("dtor PID\n");
}


} // namespace control
