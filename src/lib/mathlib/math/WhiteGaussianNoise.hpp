/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file WhiteGaussianNoise.hpp
 *
 * Shared white Gaussian noise generator for simulation modules.
 */

#pragma once

#include <matrix/math.hpp>
#include <stddef.h>

namespace math
{

/**
 * Fill out[0..n) with independent white Gaussian noise samples with zero mean and unit standard
 * deviation.
 *
 * Polar Box-Muller on top of rand(); seed with srand() once before first use. Each accepted draw
 * yields two samples, so prefer one call for n samples over n scalar calls. Stateless apart from
 * rand() itself, so it is safe to call from several threads.
 */
void generate_wgn(float *out, size_t n);

/**
 * Generate a single white Gaussian noise sample with zero mean and unit standard deviation.
 */
float generate_wgn();

/**
 * Generate a vector of three independent zero-mean Gaussian samples with the given standard deviations.
 */
matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

} // namespace math
