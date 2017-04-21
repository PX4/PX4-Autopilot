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

/// @file	FOAWFilter.cpp
/// @brief	A class to implement a first order adaptive windowing filter for derivative smoothing 
/// Author: Mathieu Bresciani <brescianimathieu@gmail.com>

#include <px4_defines.h>
#include "FOAWDifferentiator.hpp"
#include <cmath>

namespace math
{
    FOAWDifferentiator::FOAWDifferentiator(float dt, float noise_level)
    {
        set_noise_level(noise_level);
        set_sample_time(dt);
        reset();
    }

    FOAWDifferentiator::~FOAWDifferentiator()
    {

    }

    void FOAWDifferentiator::set_noise_level(float noise_level)
    {
        _delta = noise_level;
    }

    void FOAWDifferentiator::set_sample_time(float dt)
    {
        _dt = dt;
    }

    uint8_t FOAWDifferentiator::get_last_window_size(void)
    {
        return _last_window_size;
    }

   void FOAWDifferentiator::reset(void) 
    {
        memset(&_buffer, 0, sizeof(_buffer));
        _nb_samples = 0;
        _last_window_size = 0;
    }
    
    void FOAWDifferentiator::add_sample(float sample)
    {
        if (_nb_samples < _max_window_size) {
            _nb_samples++;
        }
        else{
            shift_buffer();
            _nb_samples = _max_window_size;
        }

        _buffer[_nb_samples-1] = sample;
    }

    void FOAWDifferentiator::shift_buffer(void)
    {
        for (int i = 0; i < (_nb_samples-1); i++) {
           _buffer[i] = _buffer[i+1]; 
        }

    }
 
    float FOAWDifferentiator::end_fit_FOAW(uint8_t window_size)
    {
        float result;
        float d_amplitude;
        float d_time;
        uint8_t last_sample_pos;

        last_sample_pos = _nb_samples - 1;

        d_amplitude = _buffer[last_sample_pos] - _buffer[last_sample_pos-window_size];
        d_time  = window_size * _dt;
        result = d_amplitude / d_time;

        return result;
    }

    float FOAWDifferentiator::best_fit_FOAW(uint8_t window_size)
    {
        // TODO: Implement best fit algorithm (least squares fit)
        return 0.0f;
    }

    float FOAWDifferentiator::fit(void)
    {
        uint8_t window_size;
        uint8_t j;
        uint8_t last_sample_pos;
        uint8_t pass;
        float pos;
        float result;
        float slope;

        last_sample_pos = _nb_samples - 1;
        pass = 0;
        pos = 0.0f;
        result = 0.0f;
        slope = 0.0f;
        window_size = 1;

        slope = end_fit_FOAW(window_size); 
        result = slope;

        if (last_sample_pos == 0) {
            return 0.0f;
        }

        for (window_size = 2; window_size <= (_nb_samples-1); window_size++) {
            slope = end_fit_FOAW(window_size);

            // Check if all the values are around the fit +/- delta
            for (j = 1; j < window_size; j++) {
                // Compute a point on the slope
                pos = _buffer[last_sample_pos] - slope*j*_dt;

                // Compute min and max bounds
                float max_bound = pos + _delta;
                float min_bound = pos - _delta;
                // Select sample to check
                float sample_to_check = _buffer[last_sample_pos-j];

                // Pass the test if the sample is inside the boundaries
                if (sample_to_check <= max_bound  && sample_to_check >= min_bound) {
                    pass++;
                }
                else {
                    break;
                }
            }

            // If all the values inside the windw are inside the boundaries, accept the new slope and continue with a bigger window if possible
            if (pass == (window_size-1)) {
                result = slope;
                pass = 0;
                _last_window_size = window_size;
            }
            // Otherwise (at least one sample is outside) we keep the previous slope
            else {
                break;
            }

        }
        return result;
    }


    float FOAWDifferentiator::apply(float sample)
    {
        float derivative;

        add_sample(sample);
        derivative = fit();

        return derivative;
    }



} // namespace math

