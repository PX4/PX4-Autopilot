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
/*
 * @file	FOAWDifferentiator.hpp
 * @brief	A class to implement a first order adaptive windowing differentiator 
 * Author: Mathieu Bresciani <brescianimathieu@gmail.com>
 * From:  Discrete-Time Adaptive Windowing for Velocity Estimation
 * Farrokh Janabi-Sharifi, Vincent Hayward, and Chung-Shin J. Chen
 */

#pragma once

namespace math
{
class __EXPORT FOAWDifferentiator
{
public:
    // Constructor
    FOAWDifferentiator(float sample_time, float noise_level); 

    // Destructor
    ~FOAWDifferentiator();

    /**
     * Change the main parameter of the differenciator
     * A small value will make the differenciator more agressive and close to a raw first order derivative
     * A large value will force the differenciator to use more samples (max 15) to compute the
     * least squares slope (smoother output, but less reactive).
     */
    void set_noise_level(float delta);

    /**
     * Set the elapsed time between two measurements
     */
    void set_sample_time(float dt);

    /**
     * Add a new raw value to the filter
     *
     * @return Retrieve the filtered result
     */
    float apply(float sample);

    /**
     * @return Retrieve the noise level parameter (_delta) 
     */
    float get_noise_level(void); 

    /**
     * @return Retrieve the size of the window used to compute the derivative
     */
    uint8_t get_last_window_size(void);

    /**
     * Reset the filter
     */
    void reset(void);

private:

    /*
     * Handles the paramaters of a first order curve
     * f = a*x + b
     */
    struct fit_params{
        float a;
        float b;
    }fit_val;

    /*
     * Moves all the samples in the buffer one step forward
     * The oldest sample if thrown away
     */
    void shift_buffer(void);

    /*
     * Add a new sample at the beginning of the buffer
     */
    void add_sample(float sample);

    /*
     * Performs a fit using only the first and last values
     */
    void end_fit_FOAW(uint8_t window_size);

    /*
     * Permorms a fit based on a least squares estimate using all the samples inside the window
     */
    void best_fit_FOAW(uint8_t window_size);
    float fit(void);


    float           _dt;                            // Sample time
    float           _delta;                         // Noise level parameter 
    float           _buffer[15];                    // Buffer of samples
    uint8_t         _nb_samples;                    // Number of samples inside the buffer
    uint8_t         _last_window_size;              // Size of the window used to compute the derivative
    static const uint8_t   _max_window_size = 14;   // Maximum possible size of the window
};

} // namespace math
