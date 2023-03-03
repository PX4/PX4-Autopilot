/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * This module is a modification of the hippocampus control module and is designed for the
 * BlueROV2.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

#include "usv_omni_control.hpp"


/**
 * USV pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int usv_omni_control_main(int argc, char *argv[]);


USVOmniControl::USVOmniControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

USVOmniControl::~USVOmniControl()
{
	perf_free(_loop_perf);
}

bool USVOmniControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void USVOmniControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void USVOmniControl::poseController6dof(const Vector3f &pos_des,
					const float roll_des, const float pitch_des, const float yaw_des,
					vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);

	Vector3f p_control_output = Vector3f(
					    _param_pose_gain_x.get() * (pos_des(0) - vlocal_pos.x) - _param_pose_gain_d_x.get() * vlocal_pos.vx,
					    _param_pose_gain_y.get() * (pos_des(1) - vlocal_pos.y) - _param_pose_gain_d_y.get() * vlocal_pos.vy,
					    _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z) - _param_pose_gain_d_z.get() * vlocal_pos.vz);

	Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)

	publishAttitudeSetpoint(rotated_input,
				0.0f, 0.0f, yaw_des);

}

void USVOmniControl::stabilizationController6dof(const Vector3f &pos_des,
		const float roll_des, const float pitch_des, const float yaw_des,
		vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);

	Vector3f p_control_output = Vector3f(
					    0,
					    0,
					    _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z));
	//potential d controller missing
	Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output);//rotate the coord.sys (from global to body)
	rotated_input(0) += pos_des(0);
	rotated_input(1) += pos_des(1);
	publishAttitudeSetpoint(rotated_input,
				0.0f, 0.0f, yaw_des);

}

float normalizeJoystickThrottleInput(float manual_control_throttle)
{
	// thrust normalization for joystick input by afwilkin · Pull Request #20885 · PX4/PX4-Autopilot
	// https://github.com/PX4/PX4-Autopilot/pull/20885
	return (manual_control_throttle + 1.f) * .5f;
}

/* Custom Joystick mapping
* Yaw -> Turn right/left
* Throttle -> Forward/back
* Roll -> Sideways right/left
*/
void USVOmniControl::handleManualInputs(const manual_control_setpoint_s &manual_control_setpoint)
{
	// TODO: POSITION MODE, MANUAL MODE
	// handle all the different modes that use manual_control_setpoint
	if (_control_mode.flag_control_manual_enabled && !_control_mode.flag_control_rates_enabled) {
		if (_control_mode.flag_control_attitude_enabled) {
			/* stabilized yaw, att_sp -> controlAttitude */
			// STABILIZED mode generate the attitude setpoint from manual user inputs
			float dt = math::constrain(hrt_elapsed_time(&_manual_setpoint_last_called) * 1e-6f,  0.0002f, 0.04f);

			/* reset yaw setpoint to current position if needed */
			if (_reset_yaw_sp) {
				const float vehicle_yaw = Eulerf(Quatf(_att.q)).psi();
				_manual_yaw_sp = vehicle_yaw;
				_reset_yaw_sp = false;

			} else {
				const float yaw_rate = math::radians(_param_gnd_man_y_max.get());
				_att_sp.yaw_sp_move_rate = manual_control_setpoint.yaw * yaw_rate;
				_manual_yaw_sp = matrix::wrap_pi(_manual_yaw_sp + _att_sp.yaw_sp_move_rate * dt);
			}

			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _manual_yaw_sp;

			// TODO(not7cd): Do I need to rotate this to NED frame?
			_att_sp.thrust_body[0] = normalizeJoystickThrottleInput(manual_control_setpoint.throttle);
			_att_sp.thrust_body[1] = manual_control_setpoint.roll;
			_att_sp.thrust_body[2] = 0.0f;

			// TODO(not7cd): is this required? Can I move it to publishAttitudeSetpoint?
			Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
			q.copyTo(_att_sp.q_d);

			_att_sp.timestamp = hrt_absolute_time();
			_att_sp_pub.publish(_att_sp);
			// TODO: ??? publishAttitudeSetpoint();

		} else {
			/* direct control */
			_thrust_setpoint.setAll(0.0f);
			_thrust_setpoint(0) = normalizeJoystickThrottleInput(manual_control_setpoint.throttle);
			_thrust_setpoint(1) = manual_control_setpoint.roll;

			_torque_setpoint.setAll(0.0f);
			_torque_setpoint(2) = manual_control_setpoint.yaw;
		}
	}
}

void USVOmniControl::handleVelocityInputs()
{
}

void USVOmniControl::handlePositionInputs(
	const matrix::Vector2d &current_position,
	const matrix::Vector3f &ground_speed,
	const position_setpoint_triplet_s &pos_sp_triplet)
{
}



bool USVOmniControl::controlPosition(
	const matrix::Vector2d &global_pos, const matrix::Vector3f &ground_speed,
	const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if ((_control_mode.flag_control_auto_enabled ||
	     _control_mode.flag_control_offboard_enabled) && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one currently heading for) */
		matrix::Vector2d curr_wp(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2d prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = pos_sp_triplet.previous.lat;
			prev_wp(1) = pos_sp_triplet.previous.lon;
		}

		matrix::Vector2f ground_speed_2d(ground_speed);

		float mission_throttle = _param_throttle_cruise.get();

		/* Just control the throttle */
		if (_param_speed_control_mode.get() == 1) {
			/* control the speed in closed loop */

			float mission_target_speed = _param_gndspeed_trim.get();

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
			    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _vehicle_acceleration_sub.get().xyz[0];

			// Compute airspeed control out and just scale it as a constant
			mission_throttle = _param_throttle_speed_scaler.get()
					   * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle, _param_throttle_min.get(), _param_throttle_max.get());

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist_target = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				    (double)curr_wp(0), (double)curr_wp(1)); // pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		PX4_INFO("Setpoint type %d", (int) pos_sp_triplet.current.type);
		PX4_INFO(" State machine state %d", (int) _pos_ctrl_state);
		PX4_INFO(" Setpoint Lat %f, Lon %f", (double) curr_wp(0), (double)curr_wp(1));
		PX4_INFO(" Distance to target %f", (double) dist_target);

		switch (_pos_ctrl_state) {
		case GOTO_WAYPOINT: {
				if (dist_target < _param_nav_loiter_rad.get()) {
					_pos_ctrl_state = STOPPING;  // We are closer than loiter radius to waypoint, stop.

				} else {
					// matrix::Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
					// matrix::Vector2f curr_wp_local = _global_local_proj_ref.project(curr_wp(0), curr_wp(1));
					// matrix::Vector2f prev_wp_local = _global_local_proj_ref.project(prev_wp(0),
					// 				 prev_wp(1));
					// TODO(not7cd): HOW DO I NAVIGATE?
					// _gnd_control.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed_2d);

					// TODO(not7cd): Use att_sp?
					// _act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;

					// float desired_r = ground_speed_2d.norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
					// float desired_theta = (0.5f * M_PI_F) - atan2f(desired_r, _param_wheel_base.get());
					// float control_effort = (desired_theta / _param_max_turn_angle.get()) * sign(
					// 			       _gnd_control.nav_lateral_acceleration_demand());
					// TODO(not7cd): Use att_sp?
					// control_effort = math::constrain(control_effort, -1.0f, 1.0f);
					// _act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;
				}
			}
			break;

		case STOPPING: {
				// TODO(not7cd): Don't act_controls, let CA do it
				// _act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
				// _act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				// Note _prev_wp is different to the local prev_wp which is related to a mission waypoint.
				float dist_between_waypoints = get_distance_to_next_waypoint((double)_prev_wp(0), (double)_prev_wp(1),
							       (double)curr_wp(0), (double)curr_wp(1));

				if (dist_between_waypoints > 0) {
					_pos_ctrl_state = GOTO_WAYPOINT; // A new waypoint has arrived go to it
				}

				PX4_INFO(" Distance between prev and curr waypoints %f", (double)dist_between_waypoints);
			}
			break;

		default:
			PX4_ERR("Unknown Rover State");
			_pos_ctrl_state = STOPPING;
			break;
		}

		_prev_wp = curr_wp;

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

void USVOmniControl::controlVelocity(const matrix::Vector3f &current_velocity)
{
	// TODO(not7cd): do something with trajectory setpoint here
}

void USVOmniControl::controlAttitude(
	const vehicle_attitude_s &attitude,
	const vehicle_attitude_setpoint_s &attitude_setpoint,
	const vehicle_angular_velocity_s &angular_velocity,
	const vehicle_rates_setpoint_s &rates_setpoint)
{
	/* Geometric Controller from UUV*/
	// IT KINDA WORKS, I GUESS
	Eulerf euler_angles(matrix::Quatf(attitude.q));

	float yaw_u;
	float yaw_body = attitude_setpoint.yaw_body;
	float yaw_rate_desired = rates_setpoint.yaw;

	/* get attitude setpoint rotational matrix */
	// TODO: simplify
	Dcmf rot_des = Eulerf(0.0f, 0.0f, yaw_body);

	/* get current rotation matrix from control state quaternions */
	Quatf q_att(attitude.q);
	Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	Vector3f e_R_vec;
	Vector3f torques;

	/* Compute matrix: attitude error */
	Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5;

	/* vee-map the error to get a vector instead of matrix e_R */
	e_R_vec(0) = e_R(2, 1);  /**< Roll  */
	e_R_vec(1) = e_R(0, 2);  /**< Pitch */
	e_R_vec(2) = e_R(1, 0);  /**< Yaw   */

	Vector3f omega{angular_velocity.xyz};
	omega(2) -= yaw_rate_desired;

	torques(2) = - e_R_vec(2) * _param_yaw_p.get(); /**< P-Control */
	torques(2) = torques(2) - omega(2) * _param_yaw_d.get(); /**< PD-Control */
	yaw_u = torques(2);

	// take thrust as is
	_thrust_setpoint.setAll(0.0f);
	_thrust_setpoint(0) = attitude_setpoint.thrust_body[0];
	_thrust_setpoint(1) = attitude_setpoint.thrust_body[1];

	_torque_setpoint.setAll(0.0f);
	_torque_setpoint(0) = yaw_u;
}

void USVOmniControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	float scaling = _param_scale_torque_ca.get();
	vehicle_torque_setpoint_s result{};
	result.timestamp_sample = timestamp_sample;
	result.xyz[0] = 0.0f;
	result.xyz[1] = 0.0f;
	result.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) * scaling : 0.0f;
	result.timestamp = hrt_absolute_time();
	_vehicle_torque_setpoint_pub.publish(result);
}

void USVOmniControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	float scaling = _param_scale_thrust_ca.get();
	vehicle_thrust_setpoint_s result{};
	result.timestamp_sample = timestamp_sample;
	_thrust_setpoint.copyTo(result.xyz);
	result.xyz[0] = (PX4_ISFINITE(_thrust_setpoint(0))) ? _thrust_setpoint(0) * scaling : 0.0f;
	result.xyz[1] = (PX4_ISFINITE(_thrust_setpoint(1))) ? _thrust_setpoint(1) * scaling : 0.0f;
	result.xyz[2] = 0.0f;  // Ignore Z axis, because we are grounded
	result.timestamp = hrt_absolute_time();
	_vehicle_thrust_setpoint_pub.publish(result);
}

void USVOmniControl::publishAttitudeSetpoint(const Vector3f &thrust_body_sp,
		const float roll_des, const float pitch_des, const float yaw_des)
{
	//watch if inputs are not to high
	vehicle_attitude_setpoint_s result = {};
	result.timestamp = hrt_absolute_time();

	// This is NED frame
	result.roll_body = roll_des;
	result.pitch_body = pitch_des;
	result.yaw_body = yaw_des;

	result.thrust_body[0] = (PX4_ISFINITE(thrust_body_sp(0))) ? thrust_body_sp(0) : 0.0f;
	result.thrust_body[1] = (PX4_ISFINITE(thrust_body_sp(1))) ? thrust_body_sp(1) : 0.0f;
	result.thrust_body[2] = 0.0f; // Ignore Z axis, because we are grounded

	_att_sp_pub.publish(result);
}

void USVOmniControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vehicle_control_mode_sub.update(&_control_mode);


	/* update parameters from storage */
	parameters_update();

	//vehicle_attitude_s attitude;
	vehicle_local_position_s vlocal_pos;

	/* only run controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)) {

		/* Run geometric attitude controllers if NOT manual mode*/
		if (!_control_mode.flag_control_manual_enabled
		    && _control_mode.flag_control_attitude_enabled
		    && _control_mode.flag_control_rates_enabled) {

			_vehicle_attitude_sub.update(&_att); // current vehicle attitude
			_trajectory_setpoint_sub.update(&_trajectory_sp);

			float roll_des = 0;
			float pitch_des = 0;
			float yaw_des = _trajectory_sp.yaw;

			//stabilization controller(keep pos and hold depth + angle) vs position controller(global + yaw)
			if (_param_stabilization.get() == 0) {
				poseController6dof(Vector3f(_trajectory_sp.position),
						   roll_des, pitch_des, yaw_des, _att, vlocal_pos);

			} else {
				stabilizationController6dof(Vector3f(_trajectory_sp.position),
							    roll_des, pitch_des, yaw_des, _att, vlocal_pos);
			}
		}
	}

	/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
	manual_control_setpoint_s manual_control_sp;

	if (_manual_control_setpoint_sub.update(&manual_control_sp)) {
		// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
		// returning immediately and this loop will eat up resources.

		// NOTE: all control remapping could be done here, while handleManualInputs could become generic handler for omni vehicles
		handleManualInputs(manual_control_sp);
	}

	// Respond to an attitude update and run the attitude controller if enabled
	if (_control_mode.flag_control_attitude_enabled
	    && !_control_mode.flag_control_position_enabled
	    && !_control_mode.flag_control_velocity_enabled) {
		vehicle_angular_velocity_s angular_vel;
		vehicle_rates_setpoint_s rates_sp;

		if (_vehicle_angular_velocity_sub.update(&angular_vel) && _vehicle_rates_setpoint_sub.update(&rates_sp)) {
			controlAttitude(_att, _att_sp, angular_vel, rates_sp);

		} else {
			PX4_ERR("Failed to pool angular velocity and rates setpoint");
		}
	}

	/* Only publish if any of the proper modes are enabled */
	if (_control_mode.flag_control_manual_enabled ||
	    _control_mode.flag_control_attitude_enabled) {
		hrt_abstime timestamp = hrt_absolute_time();
		publishTorqueSetpoint(_torque_setpoint, timestamp);
		publishThrustSetpoint(timestamp);

	}

	perf_end(_loop_perf);
}

int USVOmniControl::task_spawn(int argc, char *argv[])
{
	USVOmniControl *instance = new USVOmniControl();

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

int USVOmniControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int USVOmniControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (USV).
Publishes `actuator_controls_0` messages at a constant 250Hz.
### Implementation
Currently, this implementation supports only a few modes:
 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The usv runs missions
### Examples
CLI usage example:
$ usv_omni_control start
$ usv_omni_control status
$ usv_omni_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("usv_omni_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int usv_omni_control_main(int argc, char *argv[])
{
    return USVOmniControl::main(argc, argv);
}
