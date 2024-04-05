/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file FixedwingLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#include "FixedwingLandDetector.h"

#include <matrix/math.hpp>

namespace land_detector
{

FixedwingLandDetector::FixedwingLandDetector()
{
	// Use Trigger time when transitioning from in-air (false) to landed (true)
	_landed_hysteresis.set_hysteresis_time_from(false, _param_lndfw_trig_time.get() * 1_s);
}

bool FixedwingLandDetector::_get_landed_state()
{
	// Filtering - Set filters to 0 which is below the threshold if data invalid

	// Velocity
	if (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 1_s) {
		// Horizontal velocity complimentary filter
		const matrix::Vector2f horizontal_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy);

		if (_vehicle_local_position.v_xy_valid && horizontal_velocity.isAllFinite()) {
			_velocity_xy_filter.update(horizontal_velocity.length());
		} else {
			_velocity_xy_filter.reset(0.f);
		}

		// Vertical velocity complimentary filter
		if (_vehicle_local_position.v_z_valid && PX4_ISFINITE(_vehicle_local_position.vz)) {
			_velocity_z_filter.update(_vehicle_local_position.vz);
		} else {
			_velocity_z_filter.reset(0.f);
		}
	}

	// Airspeed
	airspeed_validated_s airspeed_validated{};
	_airspeed_validated_sub.copy(&airspeed_validated);

	// make groundspeed threshold tighter if airspeed is invalid
	float vel_xy_max_threshold = 0.7f * _param_lndfw_vel_xy_max.get();

	if (hrt_elapsed_time(&airspeed_validated.timestamp) < 1_s
		&& airspeed_validated.airspeed_sensor_measurement_valid
		&& PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)) {

		_airspeed_filter.update(airspeed_validated.true_airspeed_m_s);
		vel_xy_max_threshold = _param_lndfw_vel_xy_max.get();
	} else {
		_airspeed_filter.reset(0.f);
	}

	// Acceleration
	// A leaking lowpass prevents biases from building up, but
	// gives a mostly correct response for short impulses.
	const float acc_hor = matrix::Vector2f(_acceleration).norm();
	_xy_accel_filtered = _xy_accel_filtered * 0.8f + acc_hor * 0.18f;


	// Only trigger flight conditions if we are armed.
	if (!_armed) {
		return true;
	}

	bool landDetected = false;

	launch_detection_status_s launch_detection_status{};
	_launch_detection_status_sub.copy(&launch_detection_status);

	// force the landed state to stay landed if we're currently in the catapult/hand-launch launch process. Detect that we are in this state
	// by checking if the last publication of launch_detection_status is less than 0.5s old, and we're not yet in the flying state.
	if (_landed_hysteresis.get_state() &&  hrt_elapsed_time(&launch_detection_status.timestamp) < 500_ms
	    && launch_detection_status.launch_detection_state < launch_detection_status_s::STATE_FLYING) {
		landDetected = true;

	} else if (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 1_s) {
		// Crude land detector for fixedwing.
		landDetected = _airspeed_filter.getState() < _param_lndfw_airspd.get()
			       && _velocity_xy_filter.getState() < vel_xy_max_threshold
			       && _velocity_z_filter.getState()  < _param_lndfw_vel_z_max.get()
			       && _xy_accel_filtered < _param_lndfw_xyaccel_max.get();

	} else {
		// Control state topic has timed out and we need to assume we're landed.
		landDetected = true;
	}

	return landDetected;
}

} // namespace land_detector
