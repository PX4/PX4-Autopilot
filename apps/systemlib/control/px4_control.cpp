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

__EXPORT float getFloatParam(param_t param)
{
    float val;
    if (param_get(param,&val))
    {
        printf("error loading param: %s", param_name(param));
    }
    return val;
}

__EXPORT Named::Named(const char * name) :
    _name(name)
{
}

__EXPORT const char * Named::prependName(const char * string) 
{
    char buf[120];
    snprintf(buf,120,"%s_%s",_name,string);
    return buf;
}

__EXPORT PID::PID(const char * name) :
    Named(name),
    _handle_kP(param_find(prependName("P"))),
    _handle_kI(param_find(prependName("I"))),
    _handle_kD(param_find(prependName("D"))),
    _handle_iMin(param_find(prependName("IMIN"))),
    _handle_iMax(param_find(prependName("IMAX"))),
    _handle_fCut(param_find(prependName("FCUT")))
{
}

__EXPORT void PID::updateParams() { 
    setKP(getFloatParam(_handle_kP));
    setKI(getFloatParam(_handle_kI));
    setKD(getFloatParam(_handle_kD));
    setIMin(getFloatParam(_handle_iMin));
    setIMax(getFloatParam(_handle_iMax));
    setFCut(getFloatParam(_handle_fCut));
} 

} // namespace px4
} // namespace control
