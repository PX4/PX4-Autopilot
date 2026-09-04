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

#include <lib/geo/geo.h>

namespace land_detector
{

FixedwingLandDetector::FixedwingLandDetector()
	: _param_rwto_tkoff_handle(param_find("RWTO_TKOFF"))
{
	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_landed_hysteresis.set_hysteresis_time_from(false, _param_lndfw_trig_time.get() * 1_s);

	_launch_detection_status_pub.advertise();
}

void FixedwingLandDetector::_update_params()
{
	int32_t runway_takeoff_enabled = 0;

	if (param_get(_param_rwto_tkoff_handle, &runway_takeoff_enabled) == PX4_OK) {
		_runway_takeoff_enabled = runway_takeoff_enabled != 0;
	}
}

void FixedwingLandDetector::_update_topics()
{
	updateLaunchDetection();
}

void FixedwingLandDetector::updateLaunchDetection()
{
	// the state machine only arms on the transition to armed, so that a launch has to be detected
	// once per arming and the vehicle stays flying until it is disarmed again
	if (_armed && !_previous_armed_state) {
		_launch_detector.reset();
	}

	const hrt_abstime now = hrt_absolute_time();

	// A launch is only ever awaited while the vehicle is sitting still. Sustained speed means it is
	// already flying, or rolling for a runway takeoff, and the launch acceleration spike is never
	// going to come: give up, otherwise waiting for it would pin the vehicle to the landed state for
	// the rest of the flight. Handling noise on the acceleration and rotation rates, which is what a
	// vehicle being carried or shaken on a catapult produces, stays the job of the launch detector.
	const bool moving = _airspeed_filtered > _param_lndfw_airspd.get()
			    || _velocity_xy_filtered > _param_lndfw_vel_xy_max.get();

	// The two takeoff methods are alternatives rather than options that combine, and a takeoff rolling
	// down a runway never produces the acceleration a launch detection waits for, so runway takeoff
	// takes precedence over a launch detection left enabled alongside it.
	const bool awaiting_a_launch = _param_fw_laun_detcn_on.get() && !_runway_takeoff_enabled;

	if (!awaiting_a_launch || !_armed || !_landed_hysteresis.get_state() || moving) {
		// either no launch is awaited, or the vehicle is not sitting on the ground waiting for one
		// (it may have been armed in the air, or launched in an earlier mode)
		_launch_detector.forceSetFlyState();

	} else if (_time_last_launch_detector_update != 0) {
		const float dt = math::constrain((now - _time_last_launch_detector_update) * 1e-6f, 0.001f, 0.1f);

		// the norm of the acceleration is frame independent, so the local position estimate can be
		// used directly without rotating it into the body frame first
		const matrix::Vector3f acceleration{_vehicle_local_position.ax, _vehicle_local_position.ay,
						    _vehicle_local_position.az};

		_launch_detector.update(dt, acceleration.norm());
	}

	_time_last_launch_detector_update = now;

	launch_detection_status_s launch_detection_status;
	launch_detection_status.timestamp = now;
	launch_detection_status.launch_detection_state = _launch_detector.getLaunchDetected();
	_launch_detection_status_pub.publish(launch_detection_status);
}

bool FixedwingLandDetector::_get_landed_state()
{
	// Only trigger flight conditions if we are armed.
	if (!_armed) {
		return true;
	}

	bool landDetected = false;

	if (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 1_s) {

		float val = 0.0f;

		if (_vehicle_local_position.v_xy_valid) {
			// Horizontal velocity complimentary filter.
			val = 0.97f * _velocity_xy_filtered + 0.03f * sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
					_vehicle_local_position.vy * _vehicle_local_position.vy);
		}

		if (PX4_ISFINITE(val)) {
			_velocity_xy_filtered = val;
		}

		// Vertical velocity complimentary filter.
		val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_vehicle_local_position.vz);

		if (PX4_ISFINITE(val)) {
			_velocity_z_filtered = val;
		}

		airspeed_validated_s airspeed_validated{};
		_airspeed_validated_sub.copy(&airspeed_validated);

		const bool airspeed_from_sensor = airspeed_validated.airspeed_source == airspeed_validated_s::SOURCE_SENSOR_1
						  || airspeed_validated.airspeed_source == airspeed_validated_s::SOURCE_SENSOR_2
						  || airspeed_validated.airspeed_source == airspeed_validated_s::SOURCE_SENSOR_3;

		bool airspeed_invalid = false;

		// set _airspeed_filtered to 0 if airspeed data is invalid or not from an actual airspeed sensor
		if (!airspeed_from_sensor || !PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    || hrt_elapsed_time(&airspeed_validated.timestamp) > 1_s) {
			_airspeed_filtered = 0.0f;
			airspeed_invalid = true;

		} else {
			_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * airspeed_validated.true_airspeed_m_s;
		}

		// rotate the body-frame specific force to the earth frame and add gravity back,
		// such that a stationary vehicle measures ~0 at any attitude.
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			const matrix::Vector3f accel_earth = matrix::Quatf(vehicle_attitude.q).rotateVector(_acceleration)
							     + matrix::Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
			const float acc_hor = matrix::Vector2f(accel_earth.xy()).norm();

			// A leaking lowpass prevents biases from building up, but
			// gives a mostly correct response for short impulses.
			_xy_accel_filtered = _xy_accel_filtered * 0.8f + acc_hor * 0.18f;
		}

		// Check for angular velocity
		const float rot_vel_hor = _angular_velocity.norm();
		val = _velocity_rot_filtered * 0.95f + rot_vel_hor * 0.05f;

		if (PX4_ISFINITE(val)) {
			_velocity_rot_filtered = val;
		}

		// make groundspeed threshold tighter if airspeed is invalid
		const float vel_xy_max_threshold   = airspeed_invalid ? 0.7f * _param_lndfw_vel_xy_max.get() :
						     _param_lndfw_vel_xy_max.get();

		// only use the max rotational threshold if neither airspeed nor groundspeed can be used for landing detection
		const float max_rotation_threshold = (!_vehicle_local_position.v_xy_valid
						      && airspeed_invalid) ? math::radians(_param_lndfw_rot_max.get()) : INFINITY;

		// Crude land detector for fixedwing.
		landDetected = _airspeed_filtered         < _param_lndfw_airspd.get()
			       && _velocity_xy_filtered   < vel_xy_max_threshold
			       && _velocity_z_filtered    < _param_lndfw_vel_z_max.get()
			       && _xy_accel_filtered      < _param_lndfw_xyaccel_max.get()
			       && _velocity_rot_filtered  < max_rotation_threshold;

	} else {
		// Control state topic has timed out and we need to assume we're landed.
		landDetected = true;
	}

	// Force the landed state to stay landed if we're currently in an early state of the takeoff state machines.
	// This prevents premature transitions to in-air during the early takeoff phase.
	if (_landed_hysteresis.get_state()) {
		fixed_wing_runway_control_s fixed_wing_runway_control{};
		_fixed_wing_runway_control_sub.copy(&fixed_wing_runway_control);

		// Check if we're in catapult/hand-launch waiting state
		const bool waiting_for_catapult_launch = _launch_detector.getLaunchDetected() ==
				launch_detection_status_s::STATE_WAITING_FOR_LAUNCH;

		// Check if we're in runway takeoff early phase (throttle ramp or clamped to runway)
		const bool waiting_for_auto_runway_climbout = hrt_elapsed_time(&fixed_wing_runway_control.timestamp) < 500_ms
				&& fixed_wing_runway_control.runway_takeoff_state < fixed_wing_runway_control_s::STATE_CLIMBOUT;

		if (waiting_for_catapult_launch || waiting_for_auto_runway_climbout) {
			return true;
		}
	}

	return landDetected;
}

} // namespace land_detector
