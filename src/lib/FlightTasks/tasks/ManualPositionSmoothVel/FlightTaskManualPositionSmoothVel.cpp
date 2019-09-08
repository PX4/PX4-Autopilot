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

	for (int i = 0; i < 2; ++i) {
		_smoothing_xy[i].reset(accel_prev(i), vel_prev(i), pos_prev(i));
	}

	_smoothing_z.reset(accel_prev(2), vel_prev(2), pos_prev(2));

	_resetPositionLock();
	_initEkfResetCounters();

	return ret;
}

void FlightTaskManualPositionSmoothVel::reActivate()
{
	// The task is reacivated while the vehicle is on the ground. To detect takeoff in mc_pos_control_main properly
	// using the generated jerk, reset the z derivatives to zero
	for (int i = 0; i < 2; ++i) {
		_smoothing_xy[i].reset(0.f, _velocity(i), _position(i));
	}

	_smoothing_z.reset(0.f, 0.f, _position(2));

	_resetPositionLock();
	_initEkfResetCounters();
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
	_resetPositionLockXY();
	_resetPositionLockZ();
}

void FlightTaskManualPositionSmoothVel::_resetPositionLockXY()
{
	_position_lock_xy_active = false;
	_position_setpoint_xy_locked(0) = NAN;
	_position_setpoint_xy_locked(1) = NAN;
}

void FlightTaskManualPositionSmoothVel::_resetPositionLockZ()
{
	_position_lock_z_active = false;
	_position_setpoint_z_locked = NAN;
}

void FlightTaskManualPositionSmoothVel::_initEkfResetCounters()
{
	_initEkfResetCountersXY();
	_initEkfResetCountersZ();
}

void FlightTaskManualPositionSmoothVel::_initEkfResetCountersXY()
{
	_reset_counters.xy = _sub_vehicle_local_position->get().xy_reset_counter;
	_reset_counters.vxy = _sub_vehicle_local_position->get().vxy_reset_counter;
}

void FlightTaskManualPositionSmoothVel::_initEkfResetCountersZ()
{
	_reset_counters.z = _sub_vehicle_local_position->get().z_reset_counter;
	_reset_counters.vz = _sub_vehicle_local_position->get().vz_reset_counter;
}

void FlightTaskManualPositionSmoothVel::_updateSetpoints()
{
	// Reset trajectories if EKF did a reset
	_checkEkfResetCounters();

	// Update state
	_updateTrajectories();

	// Set max accel/vel/jerk
	// Has to be done before _updateTrajDurations()
	_updateTrajConstraints();

	// Get yaw setpont, un-smoothed position setpoints
	// Has to be done before _checkPositionLock()
	FlightTaskManualPosition::_updateSetpoints();
	_velocity_target_xy = Vector2f(_velocity_setpoint);
	_velocity_target_z = _velocity_setpoint(2);

	// Lock or unlock position
	// Has to be done before _updateTrajDurations()
	_checkPositionLock();

	// Update durations and sync XY
	_updateTrajDurations();

	// Fill the jerk, acceleration, velocity and position setpoint vectors
	_setOutputState();
}

void FlightTaskManualPositionSmoothVel::_checkEkfResetCounters()
{
	// Check if a reset event has happened.
	_checkEkfResetCountersXY();
	_checkEkfResetCountersZ();
}

void FlightTaskManualPositionSmoothVel::_checkEkfResetCountersXY()
{
	if (_sub_vehicle_local_position->get().xy_reset_counter != _reset_counters.xy) {
		_smoothing_xy[0].setCurrentPosition(_position(0));
		_smoothing_xy[1].setCurrentPosition(_position(1));
		_reset_counters.xy = _sub_vehicle_local_position->get().xy_reset_counter;
	}

	if (_sub_vehicle_local_position->get().vxy_reset_counter != _reset_counters.vxy) {
		_smoothing_xy[0].setCurrentVelocity(_velocity(0));
		_smoothing_xy[1].setCurrentVelocity(_velocity(1));
		_reset_counters.vxy = _sub_vehicle_local_position->get().vxy_reset_counter;
	}
}

void FlightTaskManualPositionSmoothVel::_checkEkfResetCountersZ()
{
	if (_sub_vehicle_local_position->get().z_reset_counter != _reset_counters.z) {
		_smoothing_z.setCurrentPosition(_position(2));
		_reset_counters.z = _sub_vehicle_local_position->get().z_reset_counter;
	}

	if (_sub_vehicle_local_position->get().vz_reset_counter != _reset_counters.vz) {
		_smoothing_z.setCurrentVelocity(_velocity(2));
		_reset_counters.vz = _sub_vehicle_local_position->get().vz_reset_counter;
	}
}

void FlightTaskManualPositionSmoothVel::_updateTrajectories()
{
	_updateTrajectoriesXY();
	_updateTrajectoriesZ();
}

void FlightTaskManualPositionSmoothVel::_updateTrajectoriesXY()
{
	for (int i = 0; i < 2; ++i) {
		_smoothing_xy[i].updateTraj(_deltatime);

		_traj_xy.j(i) = _smoothing_xy[i].getCurrentJerk();
		_traj_xy.a(i) = _smoothing_xy[i].getCurrentAcceleration();
		_traj_xy.v(i) = _smoothing_xy[i].getCurrentVelocity();
		_traj_xy.x(i) = _smoothing_xy[i].getCurrentPosition();
	}
}

void FlightTaskManualPositionSmoothVel::_updateTrajectoriesZ()
{
	_smoothing_z.updateTraj(_deltatime);

	_traj_z.j = _smoothing_z.getCurrentJerk();
	_traj_z.a = _smoothing_z.getCurrentAcceleration();
	_traj_z.v = _smoothing_z.getCurrentVelocity();
	_traj_z.x = _smoothing_z.getCurrentPosition();
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraints()
{
	_updateTrajConstraintsXY();
	_updateTrajConstraintsZ();
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraintsXY()
{
	for (int i = 0; i < 2; i++) {
		_smoothing_xy[i].setMaxJerk(_param_mpc_jerk_max.get());
		_smoothing_xy[i].setMaxAccel(_param_mpc_acc_hor_max.get());
		_smoothing_xy[i].setMaxVel(_constraints.speed_xy);
	}
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraintsZ()
{
	_smoothing_z.setMaxJerk(_param_mpc_jerk_max.get());

	if (_velocity_setpoint(2) < 0.f) { // up
		_smoothing_z.setMaxAccel(_param_mpc_acc_up_max.get());
		_smoothing_z.setMaxVel(_constraints.speed_up);

	} else { // down
		_smoothing_z.setMaxAccel(_param_mpc_acc_down_max.get());
		_smoothing_z.setMaxVel(_constraints.speed_down);
	}
}

void FlightTaskManualPositionSmoothVel::_updateTrajDurations()
{
	_updateTrajDurationsXY();
	_updateTrajDurationsZ();
}

void FlightTaskManualPositionSmoothVel::_updateTrajDurationsXY()
{
	for (int i = 0; i < 2; ++i) {
		_smoothing_xy[i].updateDurations(_velocity_target_xy(i));
	}

	VelocitySmoothing::timeSynchronization(_smoothing_xy, 2);
}

void FlightTaskManualPositionSmoothVel::_updateTrajDurationsZ()
{
	_smoothing_z.updateDurations(_velocity_target_z);
}

void FlightTaskManualPositionSmoothVel::_checkPositionLock()
{
	/**
	 * During a position lock -> position unlock transition, we have to make sure that the velocity setpoint
	 * is continuous. We know that the output of the position loop (part of the velocity setpoint)
	 * will suddenly become null
	 * and only the feedforward (generated by this flight task) will remain.
	 * This is why the previous input of the velocity controller
	 * is used to set current velocity of the trajectory.
	 */
	_checkPositionLockXY();
	_checkPositionLockZ();
}

void FlightTaskManualPositionSmoothVel::_checkPositionLockXY()
{
	if (_traj_xy.v.length() < 0.1f &&
	    _traj_xy.a.length() < .2f &&
	    _velocity_target_xy.length() <= FLT_EPSILON) {
		// Lock position
		_position_lock_xy_active = true;
		_position_setpoint_xy_locked = _traj_xy.x;

	} else {
		// Unlock position
		if (_position_lock_xy_active) {
			_smoothing_xy[0].setCurrentVelocity(_velocity_setpoint_feedback(
					0)); // Start the trajectory at the current velocity setpoint
			_smoothing_xy[1].setCurrentVelocity(_velocity_setpoint_feedback(1));
			_position_setpoint_xy_locked(0) = NAN;
			_position_setpoint_xy_locked(1) = NAN;
		}

		_position_lock_xy_active = false;
		_smoothing_xy[0].setCurrentPosition(_position(0));
		_smoothing_xy[1].setCurrentPosition(_position(1));
	}
}

void FlightTaskManualPositionSmoothVel::_checkPositionLockZ()
{
	if (fabsf(_traj_z.v) < 0.1f &&
	    fabsf(_traj_z.a) < .2f &&
	    fabsf(_velocity_target_z) <= FLT_EPSILON) {
		// Lock position
		_position_lock_z_active = true;
		_position_setpoint_z_locked = _traj_z.x;

	} else {
		// Unlock position
		if (_position_lock_z_active) {
			_smoothing_z.setCurrentVelocity(_velocity_setpoint_feedback(
								2)); // Start the trajectory at the current velocity setpoint
			_position_setpoint_z_locked = NAN;
		}

		_position_lock_z_active = false;
		_smoothing_z.setCurrentPosition(_position(2));
	}
}

void FlightTaskManualPositionSmoothVel::_setOutputState()
{
	_setOutputStateXY();
	_setOutputStateZ();
}

void FlightTaskManualPositionSmoothVel::_setOutputStateXY()
{
	for (int i = 0; i < 2; i++) {
		_jerk_setpoint(i) = _traj_xy.j(i);
		_acceleration_setpoint(i) = _traj_xy.a(i);
		_velocity_setpoint(i) = _traj_xy.v(i);
		_position_setpoint(i) = _position_setpoint_xy_locked(i);
	}
}

void FlightTaskManualPositionSmoothVel::_setOutputStateZ()
{
	_jerk_setpoint(2) = _traj_z.j;
	_acceleration_setpoint(2) = _traj_z.a;
	_velocity_setpoint(2) = _traj_z.v;
	_position_setpoint(2) = _position_setpoint_z_locked;
}
