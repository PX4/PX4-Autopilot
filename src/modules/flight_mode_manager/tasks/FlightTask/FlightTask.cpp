#include "FlightTask.hpp"
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

constexpr uint64_t FlightTask::_timeout;
const trajectory_setpoint_s FlightTask::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};
const vehicle_constraints_s FlightTask::empty_constraints = {0, NAN, NAN, false, {}};
const landing_gear_s FlightTask::empty_landing_gear_default_keep = {0, landing_gear_s::GEAR_KEEP, {}};

bool FlightTask::activate(const trajectory_setpoint_s &last_setpoint)
{
	_resetSetpoints();
	_setDefaultConstraints();
	initEkfResetCounters();
	_time_stamp_activate = hrt_absolute_time();
	_gear = empty_landing_gear_default_keep;
	return true;
}

void FlightTask::reActivate()
{
	// Preserve vertical velocity while on the ground to allow descending by stick for reliable land detection
	trajectory_setpoint_s setpoint_preserve_vertical{empty_trajectory_setpoint};
	setpoint_preserve_vertical.velocity[2] = _velocity_setpoint(2);
	// Start from zero acceleration to make sure no accelration measurement driven by vibration gets used on the ground
	matrix::Vector3f(0.f, 0.f, 0.f).copyTo(setpoint_preserve_vertical.acceleration);
	activate(setpoint_preserve_vertical);
}

void FlightTask::initEkfResetCounters()
{
	_reset_counters.xy = _sub_vehicle_local_position.get().xy_reset_counter;
	_reset_counters.vxy = _sub_vehicle_local_position.get().vxy_reset_counter;
	_reset_counters.z = _sub_vehicle_local_position.get().z_reset_counter;
	_reset_counters.hagl = _sub_vehicle_local_position.get().dist_bottom_reset_counter;
	_reset_counters.vz = _sub_vehicle_local_position.get().vz_reset_counter;
	_reset_counters.heading = _sub_vehicle_local_position.get().heading_reset_counter;
}

uint8_t FlightTask::availableInputs(const vehicle_local_position_s &local_position, bool home_altitude_valid,
				    bool manual_control_available, hrt_abstime now)
{
	uint8_t inputs = 0;

	if ((now - local_position.timestamp) < _timeout) {
		inputs |= LocalPosition;

		if (local_position.xy_valid) { inputs |= PositionXY; }

		if (local_position.v_xy_valid) { inputs |= VelocityXY; }

		if (local_position.z_valid) { inputs |= PositionZ; }

		if (local_position.v_z_valid) { inputs |= VelocityZ; }

		if (PX4_ISFINITE(local_position.heading)) { inputs |= Heading; }
	}

	if (manual_control_available) { inputs |= ManualControl; }

	if (home_altitude_valid) { inputs |= HomeAltitude; }

	return inputs;
}

bool FlightTask::updateInitialize()
{
	_time_stamp_current = hrt_absolute_time();
	_deltatime  = math::min((_time_stamp_current - _time_stamp_last), _timeout) / 1e6f;
	_time_stamp_last = _time_stamp_current;

	_sub_vehicle_local_position.update();
	_sub_home_position.update();

	_available_inputs = availableInputs(_sub_vehicle_local_position.get(), _sub_home_position.get().valid_alt,
					    _manualControlAvailable(), _time_stamp_current);

	_evaluateVehicleLocalPosition();
	_evaluateVehicleLocalPositionSetpoint();
	_evaluateDistanceToGround();

	const uint8_t required_inputs = requiredInputs();
	return (_available_inputs & required_inputs) == required_inputs;
}

bool FlightTask::update()
{
	_checkEkfResetCounters();
	return true;
}

void FlightTask::_checkEkfResetCounters()
{
	// Check if a reset event has happened
	if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counters.xy) {
		_ekfResetHandlerPositionXY(matrix::Vector2f{_sub_vehicle_local_position.get().delta_xy});
		_reset_counters.xy = _sub_vehicle_local_position.get().xy_reset_counter;
	}

	if (_sub_vehicle_local_position.get().vxy_reset_counter != _reset_counters.vxy) {
		_ekfResetHandlerVelocityXY(matrix::Vector2f{_sub_vehicle_local_position.get().delta_vxy});
		_reset_counters.vxy = _sub_vehicle_local_position.get().vxy_reset_counter;
	}

	if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counters.z) {
		_ekfResetHandlerPositionZ(_sub_vehicle_local_position.get().delta_z);
		_reset_counters.z = _sub_vehicle_local_position.get().z_reset_counter;
	}

	if (_sub_vehicle_local_position.get().dist_bottom_reset_counter != _reset_counters.hagl) {
		_ekfResetHandlerHagl(_sub_vehicle_local_position.get().delta_dist_bottom);
		_reset_counters.hagl = _sub_vehicle_local_position.get().dist_bottom_reset_counter;

		_dist_to_bottom = NAN;
	}

	if (_sub_vehicle_local_position.get().vz_reset_counter != _reset_counters.vz) {
		_ekfResetHandlerVelocityZ(_sub_vehicle_local_position.get().delta_vz);
		_reset_counters.vz = _sub_vehicle_local_position.get().vz_reset_counter;
	}

	if (_sub_vehicle_local_position.get().heading_reset_counter != _reset_counters.heading) {
		_ekfResetHandlerHeading(_sub_vehicle_local_position.get().delta_heading);
		_reset_counters.heading = _sub_vehicle_local_position.get().heading_reset_counter;
	}
}

const trajectory_setpoint_s FlightTask::getTrajectorySetpoint()
{
	trajectory_setpoint_s trajectory_setpoint{};
	trajectory_setpoint.timestamp = hrt_absolute_time();

	_position_setpoint.copyTo(trajectory_setpoint.position);
	_velocity_setpoint.copyTo(trajectory_setpoint.velocity);
	_acceleration_setpoint.copyTo(trajectory_setpoint.acceleration);
	_jerk_setpoint.copyTo(trajectory_setpoint.jerk);

	trajectory_setpoint.yaw = _yaw_setpoint;
	trajectory_setpoint.yawspeed = _yawspeed_setpoint;

	return trajectory_setpoint;
}

void FlightTask::_resetSetpoints()
{
	_position_setpoint.setNaN();
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();
	_jerk_setpoint.setNaN();
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = NAN;
}

void FlightTask::_evaluateVehicleLocalPosition()
{
	_position.setAll(NAN);
	_velocity.setAll(NAN);
	_yaw = NAN;
	_dist_to_bottom = NAN;

	if (_available_inputs & LocalPosition) {
		_unaided_yaw = _sub_vehicle_local_position.get().unaided_heading;
		_is_yaw_good_for_control = _sub_vehicle_local_position.get().heading_good_for_control;

		if (_available_inputs & Heading) {
			_yaw = _sub_vehicle_local_position.get().heading;
		}

		if (_available_inputs & PositionXY) {
			_position(0) = _sub_vehicle_local_position.get().x;
			_position(1) = _sub_vehicle_local_position.get().y;
		}

		if (_available_inputs & PositionZ) {
			_position(2) = _sub_vehicle_local_position.get().z;
		}

		// acceleration is the velocity derivative in EKF2, so it is available whenever velocity is
		if (_available_inputs & VelocityXY) {
			_velocity(0) = _sub_vehicle_local_position.get().vx;
			_velocity(1) = _sub_vehicle_local_position.get().vy;
			_acceleration(0) = _sub_vehicle_local_position.get().ax;
			_acceleration(1) = _sub_vehicle_local_position.get().ay;
		}

		if (_available_inputs & VelocityZ) {
			_velocity(2) = _sub_vehicle_local_position.get().vz;
			_acceleration(2) = _sub_vehicle_local_position.get().az;
		}

		// distance to bottom
		if (_sub_vehicle_local_position.get().dist_bottom_valid
		    && PX4_ISFINITE(_sub_vehicle_local_position.get().dist_bottom)) {
			_dist_to_bottom = _sub_vehicle_local_position.get().dist_bottom;
		}

		// global frame reference coordinates to enable conversions
		if (_sub_vehicle_local_position.get().xy_global && _sub_vehicle_local_position.get().z_global) {
			if (!_geo_projection.isInitialized()
			    || (_geo_projection.getProjectionReferenceTimestamp() != _sub_vehicle_local_position.get().ref_timestamp)) {

				_geo_projection.initReference(_sub_vehicle_local_position.get().ref_lat, _sub_vehicle_local_position.get().ref_lon,
							      _sub_vehicle_local_position.get().ref_timestamp);

				_global_local_alt0 = _sub_vehicle_local_position.get().ref_alt;
			}
		}
	}
}

void FlightTask::_evaluateVehicleLocalPositionSetpoint()
{
	vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

	// Only use data that is received within a certain timestamp
	if (_vehicle_local_position_setpoint_sub.copy(&vehicle_local_position_setpoint)
	    && (_time_stamp_current - vehicle_local_position_setpoint.timestamp) < _timeout) {
		// Inform about the input and output of the velocity controller
		// This is used to properly initialize the velocity setpoint when onpening the position loop (position unlock)
		_velocity_setpoint_feedback = matrix::Vector3f(vehicle_local_position_setpoint.vx, vehicle_local_position_setpoint.vy,
					      vehicle_local_position_setpoint.vz);
		_acceleration_setpoint_feedback = matrix::Vector3f(vehicle_local_position_setpoint.acceleration);

	} else {
		_velocity_setpoint_feedback.setAll(NAN);
		_acceleration_setpoint_feedback.setAll(NAN);
	}
}

void FlightTask::_evaluateDistanceToGround()
{
	// Altitude above ground is local z-position or altitude above home or distance sensor altitude depending on what's available
	if (PX4_ISFINITE(_dist_to_bottom)) {
		_dist_to_ground = _dist_to_bottom;

	} else if (_sub_home_position.get().valid_alt) {
		_dist_to_ground = -(_position(2) - _sub_home_position.get().z);

	} else {
		_dist_to_ground = -_position(2);

	}
}

void FlightTask::_setDefaultConstraints()
{
	_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	_constraints.want_takeoff = false;
}

bool FlightTask::_checkTakeoff()
{
	// position setpoint above the minimum altitude
	bool position_triggered_takeoff = false;

	if (PX4_ISFINITE(_position_setpoint(2))) {
		// minimal altitude either 20cm or what is necessary for correct estimation e.g. optical flow
		float min_altitude = 0.2f;
		const float min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

		if (PX4_ISFINITE(min_distance_to_ground)) {
			min_altitude = min_distance_to_ground + 0.05f;
		}

		position_triggered_takeoff = _position_setpoint(2) < (_position(2) - min_altitude);
	}

	// upwards velocity setpoint
	bool velocity_triggered_takeoff = false;

	if (PX4_ISFINITE(_velocity_setpoint(2))) {
		velocity_triggered_takeoff = _velocity_setpoint(2) < -0.3f;
	}

	return position_triggered_takeoff || velocity_triggered_takeoff;
}
