/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <geo/geo.h>

using namespace matrix;

bool FlightTaskManualAltitude::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndUpdateStickInputs();

	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	// in addition to manual require valid position and velocity in D-direction and valid yaw
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitude::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.f;
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from position/velocity
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.f;
	_stick_yaw.reset(_yaw, _unaided_yaw);
	_setDefaultConstraints();

	_updateConstraintsFromEstimator();

	return ret;
}

void FlightTaskManualAltitude::_updateConstraintsFromEstimator()
{
	if (PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_min)) {
		_min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

	} else {
		_min_distance_to_ground = -INFINITY;
	}

	if (!PX4_ISFINITE(_max_distance_to_ground) && PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_max_z)) {
		_max_distance_to_ground = _sub_vehicle_local_position.get().hagl_max_z;
	}
}

void FlightTaskManualAltitude::_scaleSticks()
{
	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_up = fminf(_param_mpc_z_vel_max_up.get(), _velocity_constraint_up);
	const float vel_max_down = fminf(_param_mpc_z_vel_max_dn.get(), _velocity_constraint_down);
	const float vel_max_z = (_sticks.getThrottleZeroCentered() < 0.f) ? vel_max_down : vel_max_up;
	_velocity_setpoint(2) = vel_max_z * -_sticks.getThrottleZeroCenteredExpo();
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// Depending on stick inputs and velocity, position is locked.
	// If not locked, altitude setpoint is set to NAN.

	// Check if user wants to break
	const bool apply_brake = fabsf(_sticks.getThrottleZeroCenteredExpo()) <= FLT_EPSILON;

	// Check if vehicle has stopped
	const bool stopped = (_param_mpc_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _param_mpc_hold_max_z.get());

	// Manage transition between use of distance to ground and distance to local origin
	// when terrain hold behaviour has been selected.
	if (_param_mpc_alt_mode.get() == 2) {
		// Use horizontal speed as a transition criteria
		float spd_xy = Vector2f(_velocity).length();

		// Use presence of horizontal stick inputs as a transition criteria
		float stick_xy = Vector2f(_sticks.getPitchRollExpo()).length();
		bool stick_input = stick_xy > 0.001f;

		if (_terrain_hold) {
			bool too_fast = spd_xy > _param_mpc_hold_max_xy.get();

			if (stick_input || too_fast || !PX4_ISFINITE(_dist_to_bottom)) {
				// Stop using distance to ground
				_terrain_hold = false;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_dist_to_ground_lock) && PX4_ISFINITE(_dist_to_bottom)) {
					_position_setpoint(2) = _position(2) - (_dist_to_ground_lock - _dist_to_bottom);

				} else {
					_position_setpoint(2) = _position(2);
					_dist_to_ground_lock = NAN;
				}
			}

		} else {
			bool not_moving = spd_xy < 0.5f * _param_mpc_hold_max_xy.get() && stopped;

			if (!stick_input && not_moving && PX4_ISFINITE(_dist_to_bottom)) {
				// Start using distance to ground
				_terrain_hold = true;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_position_setpoint(2))) {
					_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));
				}
			}
		}

	}

	if ((_param_mpc_alt_mode.get() == 1 || _terrain_hold) && PX4_ISFINITE(_dist_to_bottom)) {
		// terrain following
		_terrainFollowing(apply_brake, stopped);

	} else {
		// normal mode where height is dependent on local frame

		if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
			// lock position
			_position_setpoint(2) = _position(2);

			// Ensure that minimum altitude is respected if
			// there is a distance sensor and distance to bottom is below minimum.
			if (PX4_ISFINITE(_dist_to_bottom) && _dist_to_bottom < _min_distance_to_ground) {
				_terrainFollowing(apply_brake, stopped);

			} else {
				_dist_to_ground_lock = NAN;
			}

		} else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
			// Position is locked but check if a reset event has happened.
			// We will shift the setpoints.
			if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counter) {
				_position_setpoint(2) = _position(2);
				_reset_counter = _sub_vehicle_local_position.get().z_reset_counter;
			}

		} else  {
			// user demands velocity change
			_position_setpoint(2) = NAN;
			// ensure that maximum altitude is respected
		}
	}

	_respectMaxAltitude();
}

void FlightTaskManualAltitude::_respectMinAltitude()
{
	// Height above ground needs to be limited (flow / range-finder)
	if (PX4_ISFINITE(_dist_to_bottom) && (_dist_to_bottom < _min_distance_to_ground)) {
		// increase altitude to minimum flow distance
		_position_setpoint(2) = _position(2) - (_min_distance_to_ground - _dist_to_bottom);
	}
}

void FlightTaskManualAltitude::_terrainFollowing(bool apply_brake, bool stopped)
{
	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_ground_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_ground_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_ground_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_ground_lock = _position_setpoint(2) = NAN;
	}
}

void FlightTaskManualAltitude::_respectMaxAltitude()
{
	if (PX4_ISFINITE(_dist_to_bottom)) {

		float vel_constrained = _param_mpc_z_p.get() * (_max_distance_to_ground - _dist_to_bottom);

		if (PX4_ISFINITE(_max_distance_to_ground)) {
			_constraints.speed_up = math::constrain(vel_constrained, -_param_mpc_z_vel_max_dn.get(), _param_mpc_z_vel_max_up.get());

		} else {
			_constraints.speed_up = _param_mpc_z_vel_max_up.get();
		}

		if (_dist_to_bottom > _max_distance_to_ground && !(_sticks.getThrottleZeroCenteredExpo() < FLT_EPSILON)) {
			_velocity_setpoint(2) = math::constrain(-vel_constrained, 0.f, _param_mpc_z_vel_max_dn.get());
		}

		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}
}

void FlightTaskManualAltitude::_respectGroundSlowdown()
{
	// Interpolate descent rate between the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		const float limit_down = math::interpolate(_dist_to_ground,
					 _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					 _param_mpc_land_speed.get(), _constraints.speed_down);
		const float limit_up = math::interpolate(_dist_to_ground,
				       _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
				       _param_mpc_tko_speed.get(), _constraints.speed_up);
		_velocity_setpoint(2) = math::constrain(_velocity_setpoint(2), -limit_up, limit_down);
	}
}

void FlightTaskManualAltitude::_ekfResetHandlerHeading(float delta_psi)
{
	// Only reset the yaw setpoint when the heading is locked
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint = wrap_pi(_yaw_setpoint + delta_psi);
	}

	_stick_yaw.ekfResetHandler(delta_psi);
}

void FlightTaskManualAltitude::_ekfResetHandlerHagl(float delta_hagl)
{
	_dist_to_ground_lock = NAN;
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	_updateYawSetpoint();
	_updateXYSetpoint();
	_updateAltitudeLock();
	_respectGroundSlowdown();
}

void FlightTaskManualAltitude::_updateYawSetpoint()
{
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint,
				       _sticks.getYawExpo(), _yaw, _deltatime,
				       _unaided_yaw);
}

void FlightTaskManualAltitude::_updateXYSetpoint()
{
	_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(
					      _sticks.getPitchRoll(), _deltatime, _yaw, _yaw_setpoint);
}

void FlightTaskManualAltitude::_applyExternalAcceleration()
{
	acc_sp_external_s cmd;

	// Use copy() so the command is re-applied every cycle (not only on new data).
	if (!_acc_sp_external_sub.copy(&cmd)) {
		PX4_DEBUG("acc_sp_ext: no topic");
		return;
	}

	// F6: Require valid horizontal velocity estimate from EKF (GPS-independent IMU/drag).
	// Rejecting early prevents injecting acc when the velocity feedback used by
	// F3 (velocity limiting) is unreliable.
	if (!_sub_vehicle_local_position.get().v_xy_valid) {
		PX4_WARN("acc_sp_ext: F6 v_xy_valid=false, ignoring");
		return;
	}

	// Watchdog: check if the command is fresh.
	const uint16_t timeout_ms = (cmd.timeout_ms > 0u) ? cmd.timeout_ms : 500u;
	const hrt_abstime elapsed  = hrt_elapsed_time(&cmd.timestamp);
	const bool cmd_fresh       = (elapsed <= (hrt_abstime)timeout_ms * 1000ULL);

	if (!cmd_fresh) {
		PX4_DEBUG("acc_sp_ext: stale (elapsed=%llu us > %u ms)", (unsigned long long)elapsed, timeout_ms);

		// F2: Active brake — if drone is still moving after stream loss, decelerate
		// using the same acc limit used for normal commands.
		const Vector2f vel_xy(_velocity(0), _velocity(1));
		const float vel_mag = vel_xy.norm();

		if (vel_mag > EXT_ACC_BRAKE_VEL_THRESHOLD) {
			const float brake_acc = _param_mpc_acc_hor_ext.get();
			_acceleration_setpoint(0) = -vel_xy(0) / vel_mag * brake_acc;
			_acceleration_setpoint(1) = -vel_xy(1) / vel_mag * brake_acc;
			_position_setpoint(0)     = NAN;
			_position_setpoint(1)     = NAN;
			_velocity_setpoint(0)     = NAN;
			_velocity_setpoint(1)     = NAN;
			PX4_INFO_RAW("[ext_acc] F2 brake vel=%.2f m/s ax=%.2f ay=%.2f\n",
				     (double)vel_mag,
				     (double)_acceleration_setpoint(0),
				     (double)_acceleration_setpoint(1));
		}

		// F8: Force land — if stream has been absent for EXT_ACC_FORCE_LAND_US and
		// the pilot has not taken over, command a gentle descent.
		if (_last_valid_ext_acc_time > 0 &&
		    hrt_elapsed_time(&_last_valid_ext_acc_time) > EXT_ACC_FORCE_LAND_US) {
			_velocity_setpoint(2) = EXT_ACC_LAND_SPEED; // NED positive = down
			PX4_WARN("acc_sp_ext: F8 force land (no cmd for >30 s)");
		}

		return;
	}

	// Mark stream as alive for F8 timer.
	_last_valid_ext_acc_time = hrt_absolute_time();

	// Require valid altitude estimate (barometer-based, GPS-independent).
	if (!_sub_vehicle_local_position.get().z_valid) {
		PX4_WARN("acc_sp_ext: z_valid=false, ignoring");
		return;
	}

	// Validate acceleration fields.
	if (!PX4_ISFINITE(cmd.acceleration[0]) || !PX4_ISFINITE(cmd.acceleration[1])) {
		PX4_WARN("acc_sp_ext: NaN acceleration, ignoring");
		return;
	}

	// Clamp to configured horizontal acceleration limit.
	const float acc_limit = _param_mpc_acc_hor_ext.get();
	float ax = math::constrain(cmd.acceleration[0], -acc_limit, acc_limit);
	float ay = math::constrain(cmd.acceleration[1], -acc_limit, acc_limit);

	// F3: Velocity limiting — prevent motor overload from acc accumulation.
	// Uses EKF dead-reckoning velocity (GPS-independent, sufficient for safety limiting).
	const Vector2f vel_xy(_velocity(0), _velocity(1));
	const float vel_mag = vel_xy.norm();

	if (vel_mag > EXT_ACC_VEL_LIMIT) {
		// Hard limit exceeded: override with braking acc regardless of command.
		const float brake_acc = _param_mpc_acc_hor_ext.get();
		ax = -vel_xy(0) / vel_mag * brake_acc;
		ay = -vel_xy(1) / vel_mag * brake_acc;
		PX4_WARN("acc_sp_ext: F3 vel limit %.2f m/s, braking", (double)vel_mag);

	} else if (vel_mag > EXT_ACC_VEL_LIMIT * EXT_ACC_VEL_WARN_RATIO) {
		// Warn zone: scale down acc linearly toward zero as vel approaches limit.
		const float scale = 1.0f - (vel_mag - EXT_ACC_VEL_LIMIT * EXT_ACC_VEL_WARN_RATIO)
				    / (EXT_ACC_VEL_LIMIT * (1.0f - EXT_ACC_VEL_WARN_RATIO));
		ax *= scale;
		ay *= scale;
	}

	PX4_INFO_RAW("[ext_acc] ax=%.2f ay=%.2f vel=%.2f elapsed=%lluus\n",
		     (double)ax, (double)ay, (double)vel_mag, (unsigned long long)elapsed);

	// Apply setpoints — override stick-based XY.
	_acceleration_setpoint(0) = ax;
	_acceleration_setpoint(1) = ay;
	_position_setpoint(0)     = NAN;
	_position_setpoint(1)     = NAN;
	_velocity_setpoint(0)     = NAN;
	_velocity_setpoint(1)     = NAN;

	// Override yaw if explicitly commanded.
	if (PX4_ISFINITE(cmd.yaw)) {
		_yaw_setpoint = cmd.yaw;
	}
}

bool FlightTaskManualAltitude::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getThrottleZeroCentered() > 0.3f;
}

bool FlightTaskManualAltitude::update()
{
	bool ret = FlightTask::update();
	_updateConstraintsFromEstimator();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();
	_max_distance_to_ground = INFINITY;

	// Apply external acceleration last so it overrides stick-based XY setpoints.
	// Falls back silently to stick control when no valid command is available.
	_applyExternalAcceleration();

	return ret;
}
