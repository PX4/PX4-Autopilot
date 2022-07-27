/****************************************************************************
 *
 *   Copyright (c) 2017, 2021 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include "RoverPositionControl.hpp"
#include <lib/geo/geo.h>

using namespace matrix;

RoverPositionControl::RoverPositionControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED,  MODULE_NAME": cycle"))
{
}

RoverPositionControl::~RoverPositionControl()
{
	perf_free(_loop_perf);
}

bool
RoverPositionControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// Initialize the variables that depend on parameters
	parameters_update(true);

	return true;
}

void RoverPositionControl::parameters_update(bool force)
{
	parameter_update_s dummy_struct;

	if (_parameter_update_sub.update(&dummy_struct) || force) {
		// Update the param instances used in the module
		updateParams();

		// Update L1 position controller parameters
		_gnd_control.set_l1_damping(_param_l1_damping.get());
		_gnd_control.set_l1_period(_param_l1_period.get());
		_gnd_control.set_l1_roll_limit(math::radians(0.0f));

		// Update Velocity controller states
		pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
		pid_set_parameters(&_speed_ctrl,
				   _param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get(),
				   _param_speed_imax.get(),
				   _param_gndspeed_max.get());

		// Update rate controller state and gains
		_rate_control.setGains(matrix::Vector3f(0.0, 0.0, _param_rate_p.get()), matrix::Vector3f(0.0, 0.0, _param_rate_i.get()),
				       matrix::Vector3f(0.0, 0.0, _param_rate_d.get()));
		_rate_control.setFeedForwardGain(matrix::Vector3f(0.0, 0.0, _param_rate_ff.get()));
		_rate_control.setIntegratorLimit(matrix::Vector3f(0.0, 0.0, _param_rate_imax.get()));
	}
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void
RoverPositionControl::attitude_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
	}
}

void
RoverPositionControl::rates_setpoint_poll()
{
	if (_rates_sp_sub.updated()) {
		_rates_sp_sub.copy(&_rates_sp);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void
RoverPositionControl::vehicle_angular_acceleration_poll()
{
	if (_vehicle_angular_acceleration_sub.updated()) {
		_vehicle_angular_acceleration_sub.copy(&_vehicle_angular_acceleration);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	if (!_control_mode.flag_control_manual_enabled) {
		return;
	}

	if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

		if (!_control_mode.flag_control_offboard_enabled) {

			if (_control_mode.flag_control_attitude_enabled) {
				// STABILIZED mode generate the attitude setpoint from manual user inputs
				_att_sp.timestamp = hrt_absolute_time();

				/* reset yaw setpoint to current position if needed */
				if (_reset_yaw_sp) {
					const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
					_manual_yaw_sp = vehicle_yaw;
					_reset_yaw_sp = false;

				} else {
					const float yaw_rate = math::radians(_param_gnd_man_y_max.get());
					const float dt = math::constrain(hrt_elapsed_time(&_manual_setpoint_last_called) * 1e-6f,  0.0002f, 0.04f);
					// Roll command in positive direction (Y): clockwise, means yawrate in positive direction (Z axis)
					_att_sp.yaw_sp_move_rate = _manual_control_setpoint.y * yaw_rate;
					_manual_yaw_sp = wrap_pi(_manual_yaw_sp + _att_sp.yaw_sp_move_rate * dt);
				}

				_att_sp.roll_body = 0.0;
				_att_sp.pitch_body = 0.0;
				_att_sp.yaw_body = _manual_yaw_sp;
				_att_sp.thrust_body[0] = _manual_control_setpoint.z;

				Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);

				_attitude_sp_pub.publish(_att_sp);

			} else if (_control_mode.flag_control_rates_enabled) {
				// ACRO mode generates the rate setpoint from manual user inputs
				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.roll = 0.0;
				_rates_sp.pitch = 0.0;
				// Yawrate is scaled with the 'maximum rate' parameter
				_rates_sp.yaw = _param_rate_max.get() * _manual_control_setpoint.y;
				_rates_sp.thrust_body[0] = _manual_control_setpoint.z;
				_rates_sp_pub.publish(_rates_sp);

			} else {
				// MANUAL mode
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = 0.0f; // Nominally roll: _manual_control_setpoint.y;
				_act_controls.control[actuator_controls_s::INDEX_PITCH] = 0.0f; // Nominally pitch: -_manual_control_setpoint.x;
				_act_controls.control[actuator_controls_s::INDEX_YAW] =
					_manual_control_setpoint.y; // Nominally yaw: _manual_control_setpoint.r;
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =
					_manual_control_setpoint.z; // Set throttle from the manual throttle channel
				_reset_yaw_sp = true;
			}

		} else {
			// If in offboard mode, reset the yaw setpoint when entering manual control in the future
			_reset_yaw_sp = true;
		}

		_manual_setpoint_last_called = hrt_absolute_time();
	}
}

bool
RoverPositionControl::control_position(const matrix::Vector2d &current_position,
				       const matrix::Vector3f &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) && pos_sp_triplet.current.valid) {
		/* current waypoint (the one currently heading for) */
		const matrix::Vector2d curr_wp(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2d prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = pos_sp_triplet.previous.lat;
			prev_wp(1) = pos_sp_triplet.previous.lon;
		}

		float throttle_output = _param_throttle_cruise.get();

		// TODO : Do we really need to consider the position setpoint's cruise throttle setting? (This is only set by DO_CHANGE_SPEED)..
		if (_param_speed_control_mode.get() == 1) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
			    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			// Compute airspeed control out and just scale it as a constant
			throttle_output = _param_throttle_speed_scaler.get()
					  * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);

			// Constrain throttle between min and max
			throttle_output = math::constrain(throttle_output, _param_throttle_min.get(), _param_throttle_max.get());

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				throttle_output = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		const float dist_target = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
					  (double)curr_wp(0), (double)curr_wp(1));

		switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {
				if (dist_target < _param_nav_loiter_rad.get()) {
					_pos_ctrl_state = STOPPING;  // We are closer than loiter radius to waypoint, stop.

				} else {
					Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
					Vector2f curr_wp_local = _global_local_proj_ref.project(curr_wp(0), curr_wp(1));
					Vector2f prev_wp_local = _global_local_proj_ref.project(prev_wp(0),
								 prev_wp(1));
					// Do a 2D navigation
					_gnd_control.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed.xy());
					_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = throttle_output;

					// Desired turn radius = (current_2d_vel ^ 2) / (lateral acceleration demand)
					float desired_r = ground_speed.xy().norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
					// Ackermann turn angle and turn radius relationship is :
					// angle = atan(Wheel_base_length / (Turn_radius - Wheel_base_width / 2)), and we ignore base width.
					float desired_theta = atan2f(_param_wheel_base.get(), desired_r);

					// Positive lateral acceleration means turning right (clockwise), which is same convention as yaw control
					float control_effort = (desired_theta / math::radians(_param_max_turn_angle.get())) * sign(
								       _gnd_control.nav_lateral_acceleration_demand());
					control_effort = math::constrain(control_effort, -1.0f, 1.0f);
					_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
				}
			}
			break;

		case STOPPING: {
				_rates_sp.roll = 0.0;
				_rates_sp.pitch = 0.0;
				_rates_sp.yaw = 0.0;
				_rates_sp.thrust_body[0] = 0.0;
				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp_pub.publish(_rates_sp);
				// Note _prev_wp is different to the local prev_wp which is related to a mission waypoint.
				float dist_between_waypoints = get_distance_to_next_waypoint((double)_prev_wp(0), (double)_prev_wp(1),
							       (double)curr_wp(0), (double)curr_wp(1));

				if (dist_between_waypoints > 0) {
					_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				}
			}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

		_prev_wp = curr_wp;
		return true;

	} else {
		return false;
	}
}

void
RoverPositionControl::control_velocity(const matrix::Vector3f &current_velocity,
				       const trajectory_setpoint_s &trajectory_setpoint)
{
	const Vector3f desired_velocity{trajectory_setpoint.velocity};
	float dt = 0.01; // Using non zero value to a avoid division by zero

	const float throttle_output = _param_throttle_cruise.get();
	const float desired_speed = desired_velocity.norm();

	if (desired_speed > 0.01f) {
		const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
		const Vector3f vel = R_to_body * Vector3f(current_velocity(0), current_velocity(1), current_velocity(2));

		const float x_vel = vel(0);
		const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

		const float control_throttle = pid_calculate(&_speed_ctrl, desired_speed, x_vel, x_acc, dt);

		//Constrain maximum throttle to mission throttle
		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(control_throttle, 0.0f, throttle_output);

		Vector3f desired_body_velocity;

		if (_velocity_frame == VelocityFrame::NED) {
			desired_body_velocity = desired_velocity;

		} else {
			// If the frame of the velocity setpoint is unknown, assume it is in local frame
			desired_body_velocity = R_to_body * desired_velocity;
		}

		const float desired_theta = atan2f(desired_body_velocity(1), desired_body_velocity(0));
		float control_effort = desired_theta / math::radians(_param_max_turn_angle.get());
		control_effort = math::constrain(control_effort, -1.0f, 1.0f);

		_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

	} else {

		_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
		_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
	}
}

void
RoverPositionControl::control_attitude(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
{
	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = Quatf(att.q).inversed() * Quatf(att_sp.q_d);
	const Eulerf euler_sp = qe;

	_rates_sp.timestamp = hrt_absolute_time();
	_rates_sp.roll = 0.0;
	_rates_sp.pitch = 0.0;

	// Set yaw-rate setpoint proportional to yaw attitude error
	_rates_sp.yaw = euler_sp.psi() * _param_att_p.get();
	_rates_sp.thrust_body[0] = math::constrain(att_sp.thrust_body[0], 0.0f, 1.0f);
	_rates_sp_pub.publish(_rates_sp);

	control_rates(_vehicle_rates, _vehicle_angular_acceleration, _local_pos, _rates_sp);
}

void
RoverPositionControl::control_rates(const vehicle_angular_velocity_s &rates, const  vehicle_angular_acceleration_s &acc,
				    const vehicle_local_position_s &local_pos,
				    const vehicle_rates_setpoint_s &rates_sp)
{
	float dt = (_control_rates_last_called > 0) ? hrt_elapsed_time(&_control_rates_last_called) * 1e-6f : 0.01f;
	_control_rates_last_called = hrt_absolute_time();

	// Velocity in body frame
	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	const Vector3f vel_body = R_to_body * Vector3f(local_pos.vx, local_pos.vy, local_pos.vz);

	// Use forward speed only as for rovers horizontal / vertical velocity is noisy / minimal
	const float forward_speed = vel_body(0);
	bool lock_integrator = bool(forward_speed < _param_rate_i_minspeed.get());

	const matrix::Vector3f vehicle_rates(rates.xyz[0], rates.xyz[1], rates.xyz[2]);
	const matrix::Vector3f rates_setpoint(rates_sp.roll, rates_sp.pitch, rates_sp.yaw);
	const matrix::Vector3f angular_acceleration{acc.xyz};

	// Calculate the required torque to achieve rate setpoint
	const matrix::Vector3f torque = _rate_control.update(vehicle_rates, rates_setpoint, angular_acceleration, dt,
					lock_integrator);

	// Effectiveness of steering control depends on the speed of the boat. This should ideally be done in the ActuatorEffectivenessMatrix side
	// but for now we manually scale it here to dynamically adjust the yaw actuator control input.

	// With a given deflection 'a' in [-1, 1] range, an actual torque applied to the boat is approximately:
	// 1/2 * Cl(Lift coefficient) * rho(Water density) * v^2 (forward speed) * a (deflection)
	// Therefore, we need to scale down the actuator control by a factor of v^-2 (decreases as speed goes up).
	// NOTE: We assume that the PID values are valid for a 'trim' speed. So a scalar will be based off of that speed reference!

	// There needs to be a sane lower boundary for the forward speed. And if we are lower than this speed, the scalar shouldn't go exponential.
	// We set this speed limit to 50% of the trim speed. Which would produce the scalar of '4' = ((1 / 0.5)^2).
	// Without this lower limit, division of the forward speed will result in scalar explosion in the range of 100 ~ 200x, which doesn't make sense.
	const float forward_speed_clipped = max(forward_speed, _param_gndspeed_trim.get() / 2);
	const float yaw_control_scalar = sq(_param_gndspeed_trim.get()) / sq(forward_speed_clipped);

	// Steering control is set directly as torque command. This will then be linearly be mapped
	// into sterring output in control_allocator.
	const float steering_input = math::constrain(torque(2) * yaw_control_scalar, -1.0f, 1.0f);

	_act_controls.control[actuator_controls_s::INDEX_YAW] = steering_input;
	_act_controls.control[actuator_controls_s::INDEX_THROTTLE] =  math::constrain(rates_sp.thrust_body[0], 0.0f, 1.0f);

	// Update debugging values
	vel_body.copyTo(_rover_pos_control_pub.get().velocity_body);
	_rover_pos_control_pub.get().raw_yaw_torque_setpoint = torque(2);
	_rover_pos_control_pub.get().yaw_control_scalar = yaw_control_scalar;
	_rover_pos_control_pub.get().raw_steering_input = torque(2) * yaw_control_scalar;
}


void
RoverPositionControl::Run()
{
	/* Run controller only on vehicle angular velocity updates*/
	if (!_vehicle_angular_velocity_sub.update(&_vehicle_rates)) {
		return;
	}

	parameters_update();
	vehicle_angular_acceleration_poll();
	vehicle_control_mode_poll();
	vehicle_attitude_poll();
	_vehicle_acceleration_sub.update();
	attitude_setpoint_poll();
	rates_setpoint_poll();

	// For stabilized / acro modes, update the attitude / rate setpoints from control setpoint
	// This over-writes the setpoints received through uORB subscription if applicable
	manual_control_setpoint_poll();

	// Update position / velocity controller only if local position & velocity was updated and we are not in manual control mode
	if (_local_pos_sub.update(&_local_pos) && !_control_mode.flag_control_manual_enabled) {

		position_setpoint_triplet_poll();
		_global_pos_sub.update(&_global_pos);
		_trajectory_setpoint_sub.update(&_trajectory_setpoint);

		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)) {

			_global_local_proj_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon,
							     _local_pos.ref_timestamp);

			_global_local_alt0 = _local_pos.ref_alt;
		}

		// Convert Local setpoints to global setpoints for offboard control
		// TODO: Remove dependency on position setpoint triplet
		if (_control_mode.flag_control_offboard_enabled) {
			_global_local_proj_ref.reproject(
				_trajectory_setpoint.position[0], _trajectory_setpoint.position[1],
				_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

			_pos_sp_triplet.current.alt = _global_local_alt0 - _trajectory_setpoint.position[2];
			_pos_sp_triplet.current.valid = true;
		}

		const matrix::Vector3f ground_speed(_local_pos.vx, _local_pos.vy, _local_pos.vz);
		const matrix::Vector2d current_position(_global_pos.lat, _global_pos.lon);

		// Position control
		if (_control_mode.flag_control_position_enabled) {

			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				//TODO: check if radius makes sense here
				float turn_distance = _param_l1_distance.get();

				// publish status
				position_controller_status_s pos_ctrl_status{};

				pos_ctrl_status.nav_roll = 0.0f;
				pos_ctrl_status.nav_pitch = 0.0f;
				pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

				pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
				pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

				pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
							  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

				pos_ctrl_status.acceptance_radius = turn_distance;
				pos_ctrl_status.yaw_acceptance = NAN;

				pos_ctrl_status.timestamp = hrt_absolute_time();

				_pos_ctrl_status_pub.publish(pos_ctrl_status);
			}

		} else if (_control_mode.flag_control_velocity_enabled) {
			// Velocity control
			control_velocity(ground_speed, _trajectory_setpoint);
		}
	}

	// Attitude control
	if (_control_mode.flag_control_attitude_enabled
	    && !_control_mode.flag_control_position_enabled
	    && !_control_mode.flag_control_velocity_enabled) {
		control_attitude(_vehicle_att, _att_sp);
	}

	// Rate control
	if (_control_mode.flag_control_rates_enabled
	    && !_control_mode.flag_control_attitude_enabled
	    && !_control_mode.flag_control_position_enabled
	    && !_control_mode.flag_control_velocity_enabled) {
		control_rates(_vehicle_rates, _vehicle_angular_acceleration, _local_pos, _rates_sp);
	}

	/* Only publish if any of the proper modes are enabled */
	if (_control_mode.flag_control_velocity_enabled ||
	    _control_mode.flag_control_attitude_enabled ||
	    _control_mode.flag_control_position_enabled ||
	    _control_mode.flag_control_manual_enabled) {
		// Log actuator controls value
		_act_controls.timestamp = hrt_absolute_time();
		_actuator_controls_pub.publish(_act_controls);

		// Output thrust / torque setpoint for control_allocator module
		vehicle_thrust_setpoint_s v_thrust_sp{};
		v_thrust_sp.timestamp = hrt_absolute_time();
		v_thrust_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_THROTTLE];
		v_thrust_sp.xyz[1] = 0.0f;
		v_thrust_sp.xyz[2] = 0.0f;
		_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

		vehicle_torque_setpoint_s v_torque_sp{};
		v_torque_sp.timestamp = hrt_absolute_time();
		v_torque_sp.xyz[0] = _act_controls.control[actuator_controls_s::INDEX_ROLL];
		v_torque_sp.xyz[1] = _act_controls.control[actuator_controls_s::INDEX_PITCH];
		v_torque_sp.xyz[2] = _act_controls.control[actuator_controls_s::INDEX_YAW];
		_vehicle_torque_setpoint_pub.publish(v_torque_sp);

		// Publish Rover Pos Control debugging uORB message
		_rover_pos_control_pub.get().timestamp = hrt_absolute_time();
		_rover_pos_control_pub.update();
	}
}

int RoverPositionControl::task_spawn(int argc, char *argv[])
{
	RoverPositionControl *instance = new RoverPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void RoverPositionControl::rate_control_status()
{
	// Print out Yaw rate control related values
	// Use 'double' datatype to print in PX4_INFO
	const double p_gain = _rate_control.get_P_gains()(2);
	const double i_gain = _rate_control.get_I_gains()(2);
	const double d_gain = _rate_control.get_D_gains()(2);
	const double ff_gain = _rate_control.get_FF_gains()(2);
	const double i_term = _rate_control.get_I_terms()(2);
	const double rate_sp = _rate_control.get_last_rate_setpoints()(2);
	const double torque_output = _rate_control.get_lat_torque_setpoint_outputs()(2);

	PX4_INFO("Yaw rate control status");
	PX4_INFO("P: %f, I: %f, D: %f, FF: %f", p_gain, i_gain, d_gain, ff_gain);
	PX4_INFO("I term: %f, Rate sp: %f, Torque op: %f", i_term, rate_sp, torque_output);
}

int RoverPositionControl::custom_command(int argc, char *argv[])
{
	if (argc > 0) {
		// rover_pos_control rate_control_status
		if (strcmp(argv[0], "rate_control_status") == 0) {
			get_instance() -> rate_control_status();
			return 0;
		}
	}

	return print_usage("unknown command");
}

int RoverPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the position of a ground rover using an L1 controller.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int rover_pos_control_main(int argc, char *argv[])
{
	return RoverPositionControl::main(argc, argv);
}
