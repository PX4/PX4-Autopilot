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

#include "px4_control.h"

namespace control
{
namespace px4
{

bool isValid(param_t param)
{
    return param != PARAM_INVALID;
}

float getFloatParam(param_t param)
{
    float val = 0;
    if (isValid(param)) 
    {
        return 0;
    }
    if (param_get(param,&val))
    {
        printf("error loading param: %s\n", param_name(param));
    }
    return val;
}

param_t findParam(const char * name)
{
    param_t param = param_find(name);
    if (param == PARAM_INVALID)
    {
        printf("error finding param: %s\n", name);
    }
    return param;
}


PID::PID(const char * name) :
    control::PID(name),
    _handle_kP(findParam(prependName("P"))),
    _handle_kI(findParam(prependName("I"))),
    _handle_kD(findParam(prependName("D"))),
    _handle_iMin(findParam(prependName("IMIN"))),
    _handle_iMax(findParam(prependName("IMAX"))),
    _handle_fCut(findParam(prependName("FCUT")))
{
}

void PID::updateParams() { 
    if (isValid(_handle_kP)) setKP(getFloatParam(_handle_kP));
    if (isValid(_handle_kI)) setKI(getFloatParam(_handle_kI));
    if (isValid(_handle_kD)) setKD(getFloatParam(_handle_kD));
    if (isValid(_handle_iMin)) setIMin(getFloatParam(_handle_iMin));
    if (isValid(_handle_iMax)) setIMax(getFloatParam(_handle_iMax));
    if (isValid(_handle_fCut)) setFCut(getFloatParam(_handle_fCut));
} 

} // namespace px4
} // namespace control
