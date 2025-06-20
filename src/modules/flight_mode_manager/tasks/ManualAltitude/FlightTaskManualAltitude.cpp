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
	const float vel_max_z = (_sticks.getPosition()(2) > 0.0f) ? vel_max_down : vel_max_up;
	_velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// - check if sticks are released (braking)
	// - check if no vertical motion (altitude lock) (noisy baro, increase default threshold)
	// - check if horizontal motion is within limit (altitude lock)

	// First check if user is controlling Z velocity with sticks
	if (fabsf(_sticks.getPositionExpo()(2)) > 0.001f) {
		if (PX4_ISFINITE(_position_setpoint(2)) || PX4_ISFINITE(_dist_to_bottom_lock)) {
			// PX4_INFO("Setting position sp to NAN");
			_current_mode = AltitudeMode::None;
			_position_setpoint(2) = NAN;
			_dist_to_bottom_lock = NAN;
			_constraints.lock_dist_bottom = false;
		}

		return;
	}

	switch ((AltitudeMode)_param_mpc_alt_mode.get()) {
	case AltitudeMode::AltitudeFollow: {
			// Altitude following - relative earth frame origin which may drift due to sensors
			// - No user throttle
			_altitude_follow_mode();
			break;
		}

	case AltitudeMode::TerrainFollow: {
			// Terrain following - relative to ground which changes with terrain variation
			// - distance sensor valid
			// - No user throttle

			// Cannot perform terrain follow without distance sensor
			if (PX4_ISFINITE(_dist_to_bottom)) {
				_terrain_follow_mode();

			} else {
				_altitude_follow_mode();
				break;
			}
			break;
		}

	case AltitudeMode::TerrainHold: {
			// Terrain hold - relative to ground when within thresholds
			// - distance sensor valid
			// - No user throttle
			// - XY vel low
			// - Z vel stopped

			// Cannot perform terrain hold without distance sensor
			if (PX4_ISFINITE(_dist_to_bottom)) {
				_terrain_hold_mode();

			} else {
				_altitude_follow_mode();
				break;
			}
		}

	case AltitudeMode::None: {
			// Nothing to do
			break;
		}
	}
}

void FlightTaskManualAltitude::_terrain_hold_mode()
{
	// Check if XY velocity is within limit to activate Terrain Hold
	bool xy_vel_okay = Vector2f(_velocity).length() < _param_mpc_hold_max_xy.get();

	if (!xy_vel_okay) {
		// XY_vel is above threshold, just follow altitude setpoint
		// Lock the position setpoint to the current Z position estimate
		if (_current_mode != AltitudeMode::AltitudeFollow) {
			_current_mode = AltitudeMode::AltitudeFollow;

			PX4_INFO("Locking to Z estimate");
			_position_setpoint(2) = _position(2);
			_dist_to_bottom_lock = NAN;
			_constraints.lock_dist_bottom = false;
		}

		return;
	}

	if (_current_mode != AltitudeMode::TerrainHold) {
		// Set velocity setpoint to 0, switch into TerrainHold
		_current_mode = AltitudeMode::TerrainHold;
		_velocity_setpoint(2) = 0.f;
		PX4_INFO("setting terrain hold");
		return;
	}

	if (!PX4_ISFINITE(_dist_to_bottom_lock)) {
		// Wait for Z to come to a stop before locking in the distance
		if (fabsf(_velocity(2)) < 0.1f) {
			_dist_to_bottom_lock = _dist_to_bottom;
			_position_setpoint(2) = _position(2);
			_constraints.lock_dist_bottom = true;
			PX4_INFO("Locking distance to %f", (double)_dist_to_bottom);
		}

		return;
	}

	// TODO:
	// - use dist_bottom_var as heuristic for distance lock

	// All criteria met
	// - distance sensor valid
	// - No user throttle
	// - XY vel low
	// - Z vel stopped
	// Update position setpoint to keep fixed distance from terrain
	float delta_distance = _dist_to_bottom - _dist_to_bottom_lock;
	_position_setpoint(2) = _position(2) + delta_distance;
}

void FlightTaskManualAltitude::_terrain_follow_mode()
{

}

void FlightTaskManualAltitude::_altitude_follow_mode()
{

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
	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_bottom_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_bottom_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_bottom_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_bottom_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_bottom_lock = _position_setpoint(2) = NAN;
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
	// PX4_INFO("_ekfResetHandlerHagl");
	_dist_to_bottom_lock = NAN;
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, _sticks.getYawExpo(), _yaw, _deltatime, _unaided_yaw);
	_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(_sticks.getPitchRoll(), _deltatime, _yaw,
				      _yaw_setpoint);
	_updateAltitudeLock();
	// _respectGroundSlowdown();
}

bool FlightTaskManualAltitude::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getPosition()(2) < -0.3f;
}

bool FlightTaskManualAltitude::update()
{
	bool ret = FlightTask::update();
	_updateConstraintsFromEstimator();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();
	_max_distance_to_ground = INFINITY;

	return ret;
}
