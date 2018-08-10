/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoTrajectoryImpl.cpp
 */

#include "FlightTaskAutoTrajectoryImpl.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

static constexpr float SIGMA_SINGLE_OP = 0.000001f;
static constexpr float BEZ_LINE_THRESHOLD = 1.0f;

FlightTaskAutoTrajectoryImpl::FlightTaskAutoTrajectoryImpl() :
	_line(nullptr, _deltatime, _position)
{}

void FlightTaskAutoTrajectoryImpl::_generateSetpoints()
{

	if (_control_points_update) {
		_update_control_points();
		_control_points_update = false;

		// for now just set it to half the acceleration because no parameter exists.
		const float deceleration = MPC_ACC_HOR.get() / 5.0f;

		float angle = 2.0f;

		// set velocity depending on the angle between the 3 waypoints
		if (Vector2f(&(_target - _prev_wp)(0)).unit_or_zero().length() > SIGMA_SINGLE_OP &&
		    Vector2f(&(_target - _next_wp)(0)).unit_or_zero().length() > SIGMA_SINGLE_OP) {

			angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
				* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
				+ 1.0f;
		}

		// the desired velocity at target
		float desired_vel = math::getVelocityFromAngle(angle, 0.5f, MPC_CRUISE_90.get(), _mc_cruise_speed);

		// compute speed at transition line - bezier based on deceleration and desired velocity at target
		bezier::BezierQuad_f bez_tmp(_pt_0, _target, _pt_1); // only care about bezier position with default dureation = 1s
		float distance_half = 0.5f * bez_tmp.getArcLength(0.1f); // half the the bezier distance
		float duration = 2.0f * (sqrtf(2.0f * deceleration * distance_half + desired_vel * desired_vel) - desired_vel) /
				 deceleration; // total time to travel bezier
		_bezier.setBezier(_pt_0, _target, _pt_1, duration);

		// if speed at intersection is larger than cruise speed, then compute duration based
		// on maximums speed at intersection.
		if (_bezier.getVelocity(0).length() > _mc_cruise_speed) {
			duration = 2.0f * (_target - _pt_0).length() / _mc_cruise_speed;
			_bezier.setBezier(_pt_0, _target, _pt_1, duration);
		}

		// straight line
		_line.setLineFromTo(_prev_wp, _pt_0);
		_line.setSpeed(_mc_cruise_speed);
		_line.setSpeedAtTarget(_bezier.getVelocity(0).length());
		_line.setAcceleration(MPC_ACC_HOR.get());
		_line.setDeceleration(deceleration);
	}

	if (_type == WaypointType::loiter && _traj_state != TrajectoryState::Bezier) {
		// vehicle requires to stop at target

		_control_points_update = true;
		_pt_0_reached_once = false;
		_line.setLineFromTo(_prev_wp, _target);
		_line.setSpeed(_mc_cruise_speed);
		_line.setSpeedAtTarget(0.0f); // minimum speed at target -> sanity number to prevent the vehicle from getting stuck
		_line.setAcceleration(MPC_ACC_HOR.get());
		// for now just set it to half the acceleration because no parameter exists.
		const float deceleration = MPC_ACC_HOR.get() / 5.0f;
		_line.setDeceleration(deceleration);
		_line.generateSetpoints(_position_setpoint, _velocity_setpoint);


	} else {
		// vehicle requires to pass target

		_pt_0_reached_once = _pt_0_reached_once || ((_pt_0 - _position_setpoint).length() < 0.1f);
		const bool pt_1_reached = (_pt_1 - _position_setpoint).length() < 0.1f;

		if (_pt_0_reached_once && !pt_1_reached) {
			Vector3f acceleration;
			_bezier.getStatesClosest(_position_setpoint, _velocity_setpoint, acceleration, _position);
			_traj_state = TrajectoryState::Bezier;

		} else if (!pt_1_reached) {
			_control_points_update = true;
			_traj_state = TrajectoryState::Line;
			_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
		}

		if (pt_1_reached) {
			_control_points_update = true;
			_pt_0_reached_once = false;
		}
	}

	if (!PX4_ISFINITE(_yaw_setpoint)) {
		_compute_heading_from_2D_vector(_yaw_setpoint, Vector2f(&_velocity_setpoint(0)));
	}
}

void FlightTaskAutoTrajectoryImpl::_update_control_points()
{
	Vector3f u_prev_to_target = (_target - _prev_wp).unit_or_zero();
	Vector3f u_target_to_next = (_next_wp - _target).unit_or_zero();

	_pt_0 = _target - (u_prev_to_target * (NAV_ACC_RAD.get() + BEZ_LINE_THRESHOLD));
	_pt_1 = _target + (u_target_to_next * (NAV_ACC_RAD.get() +  BEZ_LINE_THRESHOLD));

	Vector3f pt_0_next = _next_wp - (u_target_to_next * (NAV_ACC_RAD.get() + BEZ_LINE_THRESHOLD));

	if ((pt_0_next - _pt_1) * u_target_to_next < 0.0f) {
		// pt_0_next is closer to target than _pt_1. set _pt_1 to pt_0_next
		_pt_1 = pt_0_next;
	}
}
