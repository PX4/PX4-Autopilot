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
 * @file GaussianNoise.hpp
 *
 * Standard normal sample generators for simulation use.
 *
 * generate_wgn() draws from N(0, 1) using the Marsaglia polar method.
 *
 * Callers are responsible for seeding the process-global rand() via
 * srand() if deterministic sequences are required.
 */

#pragma once

#include <matrix/math.hpp>

namespace math
{

/**
 * Draw a sample from the standard normal distribution N(0, 1) using the
 * Marsaglia polar method (variant of Box-Muller).
 *
 * Avoids trig (cos/sin) at the cost of a rejection loop (acceptance
 * probability ~ pi/4) and amortises two rand() calls across two samples by
 * caching the second half of each iteration.
 *
 * Not thread-safe: uses shared static state (phase / V1 / V2 / S) across all
 * callers and translation units. Safe for the single-threaded / serialised
 * work-queue usage in PX4 simulation modules.
 *
 * Defined out-of-line (and marked noinline) so the rejection-loop body is
 * emitted once rather than duplicated into every call site.
 *
 * https://en.wikipedia.org/wiki/Marsaglia_polar_method
 */
float generate_wgn();

/**
 * Draw a 3D vector of independent standard normal samples, each scaled by
 * the given per-axis standard deviation. Uses generate_wgn() internally.
 */
matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz);

} // namespace math
