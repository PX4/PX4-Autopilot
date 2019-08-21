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

#include "FlightTaskManualPositionSmoothVel.hpp"

#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPositionSmoothVel::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskManualPosition::activate(last_setpoint);

	// Check if the previous FlightTask provided setpoints
	checkSetpoints(last_setpoint);
	const Vector3f accel_prev(last_setpoint.acc_x, last_setpoint.acc_y, last_setpoint.acc_z);
	const Vector3f vel_prev(last_setpoint.vx, last_setpoint.vy, last_setpoint.vz);
	const Vector3f pos_prev(last_setpoint.x, last_setpoint.y, last_setpoint.z);

	for (int i = 0; i < 3; ++i) {
		_smoothing[i].reset(accel_prev(i), vel_prev(i), pos_prev(i));
	}

	_initEkfResetCounters();
	_resetPositionLock();

	return ret;
}

void FlightTaskManualPositionSmoothVel::reActivate()
{
	// The task is reacivated while the vehicle is on the ground. To detect takeoff in mc_pos_control_main properly
	// using the generated jerk, reset the z derivatives to zero
	for (int i = 0; i < 2; ++i) {
		_smoothing[i].reset(0.f, _velocity(i), _position(i));
	}

	_smoothing[2].reset(0.f, 0.f, _position(2));

	_initEkfResetCounters();
	_resetPositionLock();
}

void FlightTaskManualPositionSmoothVel::checkSetpoints(vehicle_local_position_setpoint_s &setpoints)
{
	// If the position setpoint is unknown, set to the current postion
	if (!PX4_ISFINITE(setpoints.x)) { setpoints.x = _position(0); }

	if (!PX4_ISFINITE(setpoints.y)) { setpoints.y = _position(1); }

	if (!PX4_ISFINITE(setpoints.z)) { setpoints.z = _position(2); }

	// If the velocity setpoint is unknown, set to the current velocity
	if (!PX4_ISFINITE(setpoints.vx)) { setpoints.vx = _velocity(0); }

	if (!PX4_ISFINITE(setpoints.vy)) { setpoints.vy = _velocity(1); }

	if (!PX4_ISFINITE(setpoints.vz)) { setpoints.vz = _velocity(2); }

	// No acceleration estimate available, set to zero if the setpoint is NAN
	if (!PX4_ISFINITE(setpoints.acc_x)) { setpoints.acc_x = 0.f; }

	if (!PX4_ISFINITE(setpoints.acc_y)) { setpoints.acc_y = 0.f; }

	if (!PX4_ISFINITE(setpoints.acc_z)) { setpoints.acc_z = 0.f; }
}

void FlightTaskManualPositionSmoothVel::_resetPositionLock()
{
	_position_lock_xy_active = false;
	_position_lock_z_active = false;
	_position_setpoint_xy_locked(0) = NAN;
	_position_setpoint_xy_locked(1) = NAN;
	_position_setpoint_z_locked = NAN;
}

void FlightTaskManualPositionSmoothVel::_initEkfResetCounters()
{
	_reset_counters.xy = _sub_vehicle_local_position->get().xy_reset_counter;
	_reset_counters.vxy = _sub_vehicle_local_position->get().vxy_reset_counter;
	_reset_counters.z = _sub_vehicle_local_position->get().z_reset_counter;
	_reset_counters.vz = _sub_vehicle_local_position->get().vz_reset_counter;
}

void FlightTaskManualPositionSmoothVel::_checkEkfResetCounters()
{
	// Check if a reset event has happened.
	if (_sub_vehicle_local_position->get().xy_reset_counter != _reset_counters.xy) {
		_smoothing[0].setCurrentPosition(_position(0));
		_smoothing[1].setCurrentPosition(_position(1));
		_reset_counters.xy = _sub_vehicle_local_position->get().xy_reset_counter;
	}

	if (_sub_vehicle_local_position->get().vxy_reset_counter != _reset_counters.vxy) {
		_smoothing[0].setCurrentVelocity(_velocity(0));
		_smoothing[1].setCurrentVelocity(_velocity(1));
		_reset_counters.vxy = _sub_vehicle_local_position->get().vxy_reset_counter;
	}

	if (_sub_vehicle_local_position->get().z_reset_counter != _reset_counters.z) {
		_smoothing[2].setCurrentPosition(_position(2));
		_reset_counters.z = _sub_vehicle_local_position->get().z_reset_counter;
	}

	if (_sub_vehicle_local_position->get().vz_reset_counter != _reset_counters.vz) {
		_smoothing[2].setCurrentVelocity(_velocity(2));
		_reset_counters.vz = _sub_vehicle_local_position->get().vz_reset_counter;
	}
}

void FlightTaskManualPositionSmoothVel::_updateSetpoints()
{
	/* Get yaw setpont, un-smoothed position setpoints.*/
	FlightTaskManualPosition::_updateSetpoints();

	/* Update constraints */
	_smoothing[0].setMaxAccel(_param_mpc_acc_hor_max.get());
	_smoothing[1].setMaxAccel(_param_mpc_acc_hor_max.get());
	_smoothing[0].setMaxVel(_constraints.speed_xy);
	_smoothing[1].setMaxVel(_constraints.speed_xy);

	if (_velocity_setpoint(2) < 0.f) { // up
		_smoothing[2].setMaxAccel(_param_mpc_acc_up_max.get());
		_smoothing[2].setMaxVel(_constraints.speed_up);

	} else { // down
		_smoothing[2].setMaxAccel(_param_mpc_acc_down_max.get());
		_smoothing[2].setMaxVel(_constraints.speed_down);
	}

	float jerk[3] = {_param_mpc_jerk_max.get(), _param_mpc_jerk_max.get(), _param_mpc_jerk_max.get()};

	_checkEkfResetCounters();

	/* Check for position unlock
	 * During a position lock -> position unlock transition, we have to make sure that the velocity setpoint
	 * is continuous. We know that the output of the position loop (part of the velocity setpoint) will suddenly become null
	 * and only the feedforward (generated by this flight task) will remain. This is why the previous input of the velocity controller
	 * is used to set current velocity of the trajectory.
	 */
	Vector2f velocity_setpoint_xy = Vector2f(&_velocity_setpoint(0));

	if (velocity_setpoint_xy.length() > FLT_EPSILON) {
		if (_position_lock_xy_active) {
			_smoothing[0].setCurrentVelocity(_velocity_setpoint_feedback(
					0)); // Start the trajectory at the current velocity setpoint
			_smoothing[1].setCurrentVelocity(_velocity_setpoint_feedback(1));
			_position_setpoint_xy_locked(0) = NAN;
			_position_setpoint_xy_locked(1) = NAN;
		}

		_position_lock_xy_active = false;
	}

	if (fabsf(_velocity_setpoint(2)) > FLT_EPSILON) {
		if (_position_lock_z_active) {
			_smoothing[2].setCurrentVelocity(_velocity_setpoint_feedback(
					2)); // Start the trajectory at the current velocity setpoint
			_position_setpoint_z_locked = NAN;
		}

		_position_lock_z_active = false;
	}

	// During position lock, lower jerk to help the optimizer
	// to converge to 0 acceleration and velocity
	if (_position_lock_xy_active) {
		jerk[0] = 1.f;
		jerk[1] = 1.f;

	} else {
		jerk[0] = _param_mpc_jerk_max.get();
		jerk[1] = _param_mpc_jerk_max.get();
	}

	jerk[2] = _position_lock_z_active ? 1.f : _param_mpc_jerk_max.get();

	for (int i = 0; i < 3; ++i) {
		_smoothing[i].setMaxJerk(jerk[i]);
		_smoothing[i].updateDurations(_deltatime, _velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_smoothing, 2); // Synchronize x and y only

	if (!_position_lock_xy_active) {
		_smoothing[0].setCurrentPosition(_position(0));
		_smoothing[1].setCurrentPosition(_position(1));
	}

	if (!_position_lock_z_active) {
		_smoothing[2].setCurrentPosition(_position(2));
	}

	Vector3f pos_sp_smooth;

	for (int i = 0; i < 3; ++i) {
		_smoothing[i].integrate(_acceleration_setpoint(i), _vel_sp_smooth(i), pos_sp_smooth(i));
		_velocity_setpoint(i) = _vel_sp_smooth(i); // Feedforward
		_jerk_setpoint(i) = _smoothing[i].getCurrentJerk();
	}

	// Check for position lock transition
	if (Vector2f(_vel_sp_smooth).length() < 0.1f &&
	    Vector2f(_acceleration_setpoint).length() < .2f &&
	    velocity_setpoint_xy.length() <= FLT_EPSILON) {
		_position_lock_xy_active = true;
	}

	if (fabsf(_vel_sp_smooth(2)) < 0.1f &&
	    fabsf(_acceleration_setpoint(2)) < .2f &&
	    fabsf(_velocity_setpoint(2)) <= FLT_EPSILON) {
		_position_lock_z_active = true;
	}

	// Set valid position setpoint while in position lock.
	// When the position lock condition above is false, it does not
	// mean that the unlock condition is true. This is why
	// we are checking the lock flag here.
	if (_position_lock_xy_active) {
		_position_setpoint_xy_locked(0) = pos_sp_smooth(0);
		_position_setpoint_xy_locked(1) = pos_sp_smooth(1);

		// If the velocity setpoint is smaller than 1mm/s and that the acceleration is 0, force the setpoints
		// to zero. This is required because the generated velocity is never exactly zero and if the drone hovers
		// for a long period of time, thr drift of the position setpoint will be noticeable.
		for (int i = 0; i < 2; i++) {
			if (fabsf(_velocity_setpoint(i)) < 1e-3f && fabsf(_acceleration_setpoint(0)) < FLT_EPSILON) {
				_velocity_setpoint(i) = 0.f;
				_acceleration_setpoint(i) = 0.f;
				_smoothing[i].setCurrentVelocity(0.f);
				_smoothing[i].setCurrentAcceleration(0.f);
			}
		}
	}

	if (_position_lock_z_active) {
		_position_setpoint_z_locked = pos_sp_smooth(2);

		if (fabsf(_velocity_setpoint(2)) < 1e-3f && fabsf(_acceleration_setpoint(2)) < FLT_EPSILON) {
			_velocity_setpoint(2) = 0.f;
			_acceleration_setpoint(2) = 0.f;
			_smoothing[2].setCurrentVelocity(0.f);
			_smoothing[2].setCurrentAcceleration(0.f);
		}
	}

	_position_setpoint(0) = _position_setpoint_xy_locked(0);
	_position_setpoint(1) = _position_setpoint_xy_locked(1);
	_position_setpoint(2) = _position_setpoint_z_locked;
}
