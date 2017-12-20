
/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file VectorMath.cpp
 *
 * Simple functions for vector manipulation that do not fit into matrix lib.
 * These functions are specific for controls.
 */


#include "ControlMath.hpp"
#include <platforms/px4_defines.h>

namespace PosControl
{

/**
 * Limit vector based on a maximum tilt.
 *
 * @param vec: 3d vector in N-E-D frame
 * @param tilt_max: maximum tilt allowed
 * @return 3d vector adjusted to tilt
 *
 * Tilt is adjusted such that vector component in D-direction
 * has higher priority.
 */
matrix::Vector3f constrainTilt(const matrix::Vector3f &vec, const float &tilt_max)
{
	/* We only consider maximum tilt < 90 */
	if (tilt_max > M_PI_2_F) {
		return vec;
	}

	/* Desired tilt is above 90 -> in order to stay within tilt,
	 * vector has to be zero (N-E-D frame)*/
	if (vec(2) > 0.0f) {
		return matrix::Vector3f{};
	}

	/* Maximum tilt is 0 */
	if (tilt_max < 0.001f) {
		return matrix::Vector3f(0.0f, 0.0f, vec(2));
	}

	/* desired and maximum allowed horizontal magnitude */
	float xy_mag = matrix::Vector2f(vec(0), vec(0)).length();
	float xy_mag_max = fabsf(vec(2)) * tanf(tilt_max);

	if (xy_mag_max < xy_mag) {
		float x0 = vec(0) * xy_mag / xy_mag_max;
		float x1 = vec(1) * xy_mag / xy_mag_max;
		return matrix::Vector3f(x0, x1, vec(2));
	}

	/* No adjustment: return normal vec */
	return vec;
}

/**
 * Constrain output from PID (u-vector) with priority on altitude.
 *
 * @reference param u: PID output in N-E-D frame.
 * @reference param sat: boolean for xy and z, indicating when integration for PID
 * 					should stop: true = stop integration, false = continue integration
 * @Ulimits: Ulimits[0] = Umax, Ulimits[1] = Umin
 * @d: direction given by (r - y ); r = reference, y = measurement
 *
 * Saturation strategy:
 * u >= Umax and d >= 0 => Saturation = true
 * u >= Umax and d <= 0 => Saturation = false
 * u <= Umin and d <= 0 => Saturation = true
 * u <= Umin and d >= 0 => Saturation = false
 *
 *
 */

void constrainPIDu(matrix::Vector3f &u, bool sat[2], const float Ulimits[2], const float d[2])
{
	sat[0] = sat[1] = false;
	float xy_max = sqrtf(Ulimits[2] * Ulimits[2] - u(2) * u(2));
	float xy_mag = matrix::Vector2f(u(0), u(0)).length();


	if (u(2) * u(2) >= Ulimits[0] * Ulimits[0]) {
		/* The desired u in D-direction exceeds maximum */

		/* Check if altitude saturated
		 * negative sign bc of N-E-D frame */
		if (-d[1] >= 0.0f) {
			sat[1] = true;
		}

		sat[0] = true;
		u(0) = 0.0f;
		u(1) = 0.0f;
		u(2) = Ulimits[0];

	} if (u.length() >= Ulimits[0]) {

		/* The desired u_xy exceeds maximum */

		if (d[0] >= 0.0f) {
			sat[0] = true;
		}

		u(0) = u(0) / xy_mag * xy_max;
		u(1) = u(1) / xy_mag * xy_max;

	} else if (u.length() <= Ulimits[1]) {
		/* The desired u is below minimum */

		/* Maximum mag in xy is always reached
		 * if desired u_z is below minimum */
		sat[0] = true;

		/* Check if altitude is saturated
		 * negative sign bc of N-E-D frame */
		if (-d[1] <= 0.0f) {
			sat[1] = true;
		}

		/* If we have zero vector,
		 * then apply minimum throttle in D-direction
		 * since we do not know better. (no direction given)
		 */
		if (u.length() < 0.0001f) {
			u = matrix::Vector3f(0.0f, 0.0f, -Ulimits[1]);

		} else {
			u = u.normalized() * Ulimits[1];
		}
	}
}
}
