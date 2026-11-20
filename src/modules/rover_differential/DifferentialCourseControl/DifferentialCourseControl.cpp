/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DifferentialCourseControl.hpp"

using namespace time_literals;

DifferentialCourseControl::DifferentialCourseControl(ModuleParams *parent) :
	ModuleParams(parent),
	RoverControllerWorkItem("rover_differential_course_control", ORB_ID(vehicle_local_position))
{
	_surface_vehicle_attitude_setpoint_pub.advertise();
	_surface_vehicle_speed_setpoint_pub.advertise();

	updateParams();
}

void DifferentialCourseControl::updateParams()
{
	ModuleParams::updateParams();
}

void DifferentialCourseControl::updateCourseControl()
{
	updateSubscriptions();

	// The last valid course setpoint is controlled until a NaN course is received. A NaN course with a speed (e.g. the
	// position controller stopping at its target) keeps the current heading, so the controllers below keep getting setpoints.
	// Only a NaN course without speed (e.g. Position mode while steering) stops the course controller.
	if (!PX4_ISFINITE(_course_setpoint) && !PX4_ISFINITE(_speed_setpoint)) {
		return;
	}

	const hrt_abstime timestamp = hrt_absolute_time();
	float speed_setpoint = _speed_setpoint;

	// Without a speed setpoint the course is held with whatever thrust is commanded, the thrust sign gives the direction of travel
	const bool reverse = PX4_ISFINITE(speed_setpoint) ? speed_setpoint < -FLT_EPSILON : _thrust_setpoint_x < -FLT_EPSILON;

	float yaw_setpoint = _vehicle_yaw; // No course: keep the current heading

	if (PX4_ISFINITE(_course_setpoint)) {
		if (reverse) {
			// Driving in reverse, the rover points away from its direction of travel.
			// The drift angle is only estimated driving forwards, so it is not compensated here.
			yaw_setpoint = matrix::wrap_pi(_course_setpoint + M_PI_F);

		} else {
			// Driving forwards, the heading setpoint is the course setpoint corrected by the drift angle, like the fixed wing crabs into the wind
			yaw_setpoint = matrix::wrap_pi(_course_setpoint - _drift_angle_estimator.getDriftAngle());
		}

		const float heading_error = fabsf(matrix::wrap_pi(yaw_setpoint - _vehicle_yaw));

		if (_current_state == DrivingState::DRIVING && heading_error > _param_rd_trans_drv_trn.get()) {
			_current_state = DrivingState::SPOT_TURNING;

		} else if (_current_state == DrivingState::SPOT_TURNING && heading_error < _param_rd_trans_trn_drv.get()) {
			_current_state = DrivingState::DRIVING;
		}

		// Slow down for large heading errors and stop to turn on the spot
		if (PX4_ISFINITE(speed_setpoint)) {
			if (_current_state == DrivingState::SPOT_TURNING) {
				speed_setpoint = 0.f;

			} else if (_param_ro_speed_red.get() > FLT_EPSILON) {
				const float speed_reduction = math::constrain(_param_ro_speed_red.get() * math::interpolate(heading_error,
							      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
				const float max_speed = math::constrain(_param_ro_max_thr_speed.get() * (1.f - speed_reduction), 0.f,
									_param_ro_max_thr_speed.get());
				speed_setpoint = math::constrain(speed_setpoint, -max_speed, max_speed);
			}
		}

	} else {
		_current_state = DrivingState::DRIVING;
	}

	surface_vehicle_attitude_setpoint_s surface_vehicle_attitude_setpoint{};
	surface_vehicle_attitude_setpoint.timestamp = timestamp;
	surface_vehicle_attitude_setpoint.yaw_setpoint = yaw_setpoint;
	_surface_vehicle_attitude_setpoint_pub.publish(surface_vehicle_attitude_setpoint);

	// No speed setpoint: NaN deactivates the speed controller, the thrust then comes from elsewhere (thrust-only course hold)
	surface_vehicle_speed_setpoint_s surface_vehicle_speed_setpoint{};
	surface_vehicle_speed_setpoint.timestamp = timestamp;
	surface_vehicle_speed_setpoint.x = speed_setpoint;
	surface_vehicle_speed_setpoint.y = NAN;
	_surface_vehicle_speed_setpoint_pub.publish(surface_vehicle_speed_setpoint);
}

void DifferentialCourseControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_yaw = matrix::Eulerf(matrix::Quatf(vehicle_attitude.q)).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_drift_angle_estimator.update(vehicle_local_position);
	}

	if (_surface_vehicle_thrust_setpoint_sub.updated()) {
		surface_vehicle_thrust_setpoint_s surface_vehicle_thrust_setpoint{};
		_surface_vehicle_thrust_setpoint_sub.copy(&surface_vehicle_thrust_setpoint);
		_thrust_setpoint_x = surface_vehicle_thrust_setpoint.x;
	}

	if (_surface_vehicle_course_setpoint_sub.updated()) {
		surface_vehicle_course_setpoint_s surface_vehicle_course_setpoint{};
		_surface_vehicle_course_setpoint_sub.copy(&surface_vehicle_course_setpoint);
		_course_setpoint = surface_vehicle_course_setpoint.course;
		_speed_setpoint = surface_vehicle_course_setpoint.speed;
	}
}

bool DifferentialCourseControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_red.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
