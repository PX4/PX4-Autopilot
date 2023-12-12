#include "DifferentialDriveGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

DifferentialDriveGuidance::DifferentialDriveGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	pid_init(&_heading_p_controller, PID_MODE_DERIVATIV_NONE, 0.001f);
}

matrix::Vector2f DifferentialDriveGuidance::computeGuidance(float vehicle_yaw, float angular_velocity, float dt)
{
	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&_vehicle_global_position);
	}

	matrix::Vector2d global_position(_vehicle_global_position.lat, _vehicle_global_position.lon);
	matrix::Vector2d current_waypoint(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);
	matrix::Vector2d next_waypoint(_position_setpoint_triplet.next.lat, _position_setpoint_triplet.next.lon);

	const float distance_to_next_wp = get_distance_to_next_waypoint(global_position(0), global_position(1),
					  current_waypoint(0),
					  current_waypoint(1));

	float desired_heading = get_bearing_to_next_waypoint(global_position(0), global_position(1), current_waypoint(0),
				current_waypoint(1));
	float heading_error = matrix::wrap_pi(desired_heading - vehicle_yaw);

	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(_param_rdd_max_jerk.get(),
				   _param_rdd_max_accel.get(), distance_to_next_wp, 0.0f);

	_forwards_velocity_smoothing.updateDurations(max_velocity);
	_forwards_velocity_smoothing.updateTraj(dt);

	if (_next_waypoint != next_waypoint) {
		if (fabsf(heading_error) < 0.1f) {
			_currentState = GuidanceState::DRIVING;

		} else {
			_currentState = GuidanceState::TURNING;
		}

	}

	// Make rover stop when it arrives at the last waypoint instead of loitering and driving around weirdly.
	if ((current_waypoint == next_waypoint) && distance_to_next_wp <= _param_nav_acc_rad.get()) {
		_currentState = GuidanceState::GOAL_REACHED;
	}

	matrix::Vector2f output;
	float desired_speed = 0.f;

	switch (_currentState) {
	case GuidanceState::TURNING:
		desired_speed = 0.f;
		break;

	case GuidanceState::DRIVING:
		desired_speed = math::interpolate<float>(abs(heading_error), 0.1f, 0.4f,
				_forwards_velocity_smoothing.getCurrentVelocity(), 0.0f);
		break;

	case GuidanceState::GOAL_REACHED:
		// temporary till I find a better way to stop the vehicle
		desired_speed = 0.f;
		heading_error = 0.f;
		angular_velocity = 0.f;
		_desired_angular_velocity = 0.f;
		break;
	}

	// Compute the desired angular velocity relative to the heading error, to steer the vehicle towards the next waypoint.
	float angular_velocity_pid = pid_calculate(&_heading_p_controller, heading_error, angular_velocity, 0, dt);

	output(0) = math::constrain(desired_speed, -_max_speed, _max_speed);
	output(1) = math::constrain(angular_velocity_pid, -_max_angular_velocity, _max_angular_velocity);

	return output;
}

void DifferentialDriveGuidance::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&_heading_p_controller,
			   _param_rdd_p_gain_heading.get(),  // Proportional gain
			   0,  // Integral gain
			   0,  // Derivative gain
			   0,  // Integral limit
			   200);  // Output limit

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_max_speed);
}
