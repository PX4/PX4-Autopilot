// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

/// @file	LowPassFilter.cpp
/// @brief	A class to implement a second order low pass filter 
/// Author: Leonard Hall <LeonardTHall@gmail.com>

#include "LowPassFilter2p.hpp"
#include "math.h"

namespace math
{

void LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
    _cutoff_freq = cutoff_freq;
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/_cutoff_freq;
    float ohm = tanf(M_PI_F/fr);
    float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
    _b0 = ohm*ohm/c;
    _b1 = 2.0f*_b0;
    _b2 = _b0;
    _a1 = 2.0f*(ohm*ohm-1.0f)/c;
    _a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

float LowPassFilter2p::apply(float sample)
{
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propagate via the filter
        delay_element_0 = sample;
    }
    float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
    
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

float LowPassFilter2p::reset(float sample) {
	float dval = sample / (_b0 + _b1 + _b2);
    _delay_element_1 = dval;
    _delay_element_2 = dval;
    return apply(sample);
}

} // namespace math

