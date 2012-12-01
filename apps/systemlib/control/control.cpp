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

#include "control.h"

namespace control
{

__EXPORT Integral::Integral() :
    _x(0)
{
    printf("ctor Integral\n");
}

__EXPORT float Integral::update(float input, uint16_t dt)
{
    printf("update Integral");
    _x += input*dt;
    return _x;
}

__EXPORT Integral::~Integral()
{
    printf("dtor Integral\n");
}

__EXPORT Derivative::Derivative() :
    _x(0)
{
    printf("ctor Derivative\n");
}

__EXPORT float Derivative::update(float input, uint16_t dt)
{
    printf("update Derivative\n");
    return (input - _x)/dt;
}

__EXPORT Derivative::~Derivative()
{
    printf("dtor Derivative\n");
}

__EXPORT Limit::Limit(float min, float max) :
    _min(min), _max(max)
{
    printf("ctor Limit\n");
}

__EXPORT float Limit::update(float input, uint16_t dt)
{
    printf("update Limit\n");
    if (input > _max)
    {
        input = _max;
    }
    else if (input < _min)
    {
        input = _min;
    }
    return input;
}

__EXPORT Limit::~Limit()
{
    printf("dtor Limit\n");
}

__EXPORT PID::PID(float kP, float kI, float kD) :
    _kP(kP), _kI(kI), _kD(kD)
{
    printf("ctor PID\n");
}

__EXPORT PID::~PID()
{
    printf("dtor PID\n");
}


} // namespace control
