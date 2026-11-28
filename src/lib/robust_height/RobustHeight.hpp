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
 * @file RobustHeight.hpp
 *
 * Height above ground from the distance sensor that ignores sudden jumps, e.g. a reflection or a lost
 * return while the vehicle is tilted. The height is propagated with the estimated vertical velocity
 * and pulled towards the distance measurement no faster than RATE_MARGIN, so a jump only moves it slowly
 * while real climbs and descents (which show in the vertical velocity) are followed at once.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

class RobustHeight
{
public:
	/**
	 * @param dist_bottom distance sensor height above ground [m], NAN if invalid
	 * @param vz vertical velocity, down positive [m/s], NAN if invalid
	 * @param timestamp sample time [us]
	 * @return robust height above ground [m], NAN while the distance is invalid
	 */
	float update(float dist_bottom, float vz, hrt_abstime timestamp)
	{
		if (!PX4_ISFINITE(dist_bottom)) {
			reset();
			return NAN;
		}

		if (!PX4_ISFINITE(_height) || !PX4_ISFINITE(vz) || (timestamp <= _timestamp)) {
			_height = dist_bottom;

		} else {
			const float dt = math::min((timestamp - _timestamp) * 1e-6f, .5f);
			const float predicted = _height - vz * dt;
			_height = math::constrain(dist_bottom, predicted - RATE_MARGIN * dt, predicted + RATE_MARGIN * dt);
		}

		_timestamp = timestamp;
		return _height;
	}

	void reset()
	{
		_height = NAN;
		_timestamp = 0;
	}

	float get() const { return _height; }

private:
	static constexpr float RATE_MARGIN = .3f; ///< how fast the distance may pull the height beyond the velocity [m/s]

	float _height{NAN};
	hrt_abstime _timestamp{0};
};

/**
 * Height above the point where the vehicle was armed, from the estimated local position, for a vehicle
 * without a distance sensor (barometric height). The reference follows the vehicle while disarmed and is
 * shifted with the estimator height resets, so it stays on the ground point in flight.
 */
class ArmedHeight
{
public:
	/**
	 * @param z local position, down positive [m]
	 * @param z_valid z is valid
	 * @param z_reset_counter estimator height reset counter
	 * @param delta_z height step of the last estimator reset [m]
	 * @param armed vehicle armed
	 * @return height above the arming point [m], NAN while z is invalid or the arming point is unknown
	 */
	float update(float z, bool z_valid, uint8_t z_reset_counter, float delta_z, bool armed)
	{
		if (z_reset_counter != _z_reset_counter) {
			if (PX4_ISFINITE(_z_ground) && PX4_ISFINITE(delta_z)) {
				_z_ground += delta_z;
			}

			_z_reset_counter = z_reset_counter;
		}

		if (!z_valid || !PX4_ISFINITE(z)) {
			_height = NAN;
			return _height;
		}

		if (!armed) {
			_z_ground = z;
		}

		// Armed without a valid z at the arming point: the height stays unknown
		_height = _z_ground - z;
		return _height;
	}

	float get() const { return _height; }

private:
	float _z_ground{NAN};
	float _height{NAN};
	uint8_t _z_reset_counter{0};
};
