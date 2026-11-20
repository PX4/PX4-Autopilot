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


#include "WhiteGaussianNoise.hpp"

#include <math.h>
#include <stdlib.h>

namespace math
{

void generate_wgn(float *out, size_t n)
{
	// polar Box-Muller: every accepted draw yields two independent samples, so a vector of n
	// samples needs only ceil(n / 2) draws. All state lives on the stack, so concurrent callers
	// (sih task, sensor_*_sim work queue, gz_bridge) cannot corrupt each other's samples.
	for (size_t i = 0; i < n; i += 2) {
		float v1;
		float v2;
		float s;

		do {
			v1 = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
			v2 = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
			s = v1 * v1 + v2 * v2;
		} while (s >= 1.0f || s < 1e-8f);

		const float factor = sqrtf(-2.0f * logf(s) / s);
		out[i] = v1 * factor;

		if (i + 1 < n) {
			out[i + 1] = v2 * factor;
		}
	}
}

float generate_wgn()
{
	float x;
	generate_wgn(&x, 1);
	return x;
}

matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz)
{
	float x[3];
	generate_wgn(x, 3);
	return matrix::Vector3f(x[0] * stdx, x[1] * stdy, x[2] * stdz);
}

} // namespace math
