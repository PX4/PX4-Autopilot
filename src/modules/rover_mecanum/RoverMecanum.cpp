/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "RoverMecanum.hpp"
using namespace matrix;
using namespace time_literals;

RoverMecanum::RoverMecanum() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	_rover_mecanum_setpoint_pub.advertise();
}

bool RoverMecanum::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverMecanum::updateParams()
{
	ModuleParams::updateParams();

	_max_yaw_rate = _param_rm_max_yaw_rate.get() * M_DEG_TO_RAD_F;
}

void RoverMecanum::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();

	// Generate and publish attitude and velocity setpoints
	hrt_abstime timestamp = hrt_absolute_time();

	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
			manual_control_setpoint_s manual_control_setpoint{};
			rover_mecanum_setpoint_s rover_mecanum_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_mecanum_setpoint.timestamp = timestamp;
				rover_mecanum_setpoint.forward_speed_setpoint = NAN;
				rover_mecanum_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_mecanum_setpoint.lateral_speed_setpoint = NAN;
				rover_mecanum_setpoint.lateral_speed_setpoint_normalized = manual_control_setpoint.roll;
				rover_mecanum_setpoint.yaw_rate_setpoint = NAN;

				if (_max_yaw_rate > FLT_EPSILON && _param_rm_max_thr_yaw_r.get() > FLT_EPSILON) {
					const float scaled_yaw_rate_input = math::interpolate<float>(manual_control_setpoint.yaw,
									    -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
					const float speed_diff = scaled_yaw_rate_input * _param_rm_wheel_track.get() / 2.f;
					rover_mecanum_setpoint.speed_diff_setpoint_normalized = math::interpolate<float>(speed_diff,
							-_param_rm_max_thr_yaw_r.get(), _param_rm_max_thr_yaw_r.get(), -1.f, 1.f);

				} else {
					rover_mecanum_setpoint.speed_diff_setpoint_normalized = manual_control_setpoint.yaw;

				}

				rover_mecanum_setpoint.yaw_setpoint = NAN;
				_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);

			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO: {
			manual_control_setpoint_s manual_control_setpoint{};
			rover_mecanum_setpoint_s rover_mecanum_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_mecanum_setpoint.timestamp = timestamp;
				rover_mecanum_setpoint.forward_speed_setpoint = NAN;
				rover_mecanum_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_mecanum_setpoint.lateral_speed_setpoint = NAN;
				rover_mecanum_setpoint.lateral_speed_setpoint_normalized = manual_control_setpoint.roll;
				rover_mecanum_setpoint.yaw_rate_setpoint = math::interpolate<float>(manual_control_setpoint.yaw,
						-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_mecanum_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_mecanum_setpoint.yaw_setpoint = NAN;
				_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);
			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_STAB: {
			manual_control_setpoint_s manual_control_setpoint{};
			rover_mecanum_setpoint_s rover_mecanum_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_mecanum_setpoint.timestamp = timestamp;
				rover_mecanum_setpoint.forward_speed_setpoint = NAN;
				rover_mecanum_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_mecanum_setpoint.lateral_speed_setpoint = NAN;
				rover_mecanum_setpoint.lateral_speed_setpoint_normalized = manual_control_setpoint.roll;
				rover_mecanum_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.yaw,
						STICK_DEADZONE),
						-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_mecanum_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_mecanum_setpoint.yaw_setpoint = NAN;

				if (fabsf(rover_mecanum_setpoint.yaw_rate_setpoint) > FLT_EPSILON
				    || (fabsf(rover_mecanum_setpoint.forward_speed_setpoint_normalized) < FLT_EPSILON
					&& fabsf(rover_mecanum_setpoint.lateral_speed_setpoint_normalized) < FLT_EPSILON)) { // Closed loop yaw rate control
					_yaw_ctl = false;

				} else { // Closed loop yaw control

					if (!_yaw_ctl) {
						_desired_yaw = _vehicle_yaw;
						_yaw_ctl = true;
					}

					rover_mecanum_setpoint.yaw_setpoint = _desired_yaw;
					rover_mecanum_setpoint.yaw_rate_setpoint = NAN;
				}

				_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);
			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL: {
			manual_control_setpoint_s manual_control_setpoint{};
			rover_mecanum_setpoint_s rover_mecanum_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_mecanum_setpoint.timestamp = timestamp;
				rover_mecanum_setpoint.forward_speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
						-1.f, 1.f, -_param_rm_max_speed.get(), _param_rm_max_speed.get());;
				rover_mecanum_setpoint.forward_speed_setpoint_normalized = NAN;
				rover_mecanum_setpoint.lateral_speed_setpoint = math::interpolate<float>(manual_control_setpoint.roll,
						-1.f, 1.f, -_param_rm_max_speed.get(), _param_rm_max_speed.get());
				rover_mecanum_setpoint.lateral_speed_setpoint_normalized = NAN;
				rover_mecanum_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.yaw,
						STICK_DEADZONE),
						-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_mecanum_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_mecanum_setpoint.yaw_setpoint = NAN;

				// Reset cruise control if the manual input changes
				if (_yaw_ctl && (!(fabsf(manual_control_setpoint.throttle - _prev_throttle) > STICK_DEADZONE)
						 || !(fabsf(manual_control_setpoint.roll - _prev_roll) > STICK_DEADZONE))) {
					_yaw_ctl = false;
					_prev_throttle = manual_control_setpoint.throttle;
					_prev_roll = manual_control_setpoint.roll;

				} else if (!_yaw_ctl) {
					_prev_throttle = manual_control_setpoint.throttle;
					_prev_roll = manual_control_setpoint.roll;
				}


				if (fabsf(rover_mecanum_setpoint.yaw_rate_setpoint) > FLT_EPSILON
				    || (fabsf(rover_mecanum_setpoint.forward_speed_setpoint) < FLT_EPSILON
					&& fabsf(rover_mecanum_setpoint.lateral_speed_setpoint) < FLT_EPSILON)) { // Closed loop yaw rate control
					rover_mecanum_setpoint.yaw_rate_setpoint = math::interpolate<float>(manual_control_setpoint.yaw,
							-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
					rover_mecanum_setpoint.yaw_setpoint = NAN;
					_yaw_ctl = false;

				} else { // Cruise control
					const Vector3f velocity = Vector3f(rover_mecanum_setpoint.forward_speed_setpoint,
									   rover_mecanum_setpoint.lateral_speed_setpoint, 0.f);
					const float desired_velocity_magnitude = velocity.norm();

					if (!_yaw_ctl) {
						_desired_yaw = _vehicle_yaw;
						_yaw_ctl = true;
						_pos_ctl_start_position_ned = _curr_pos_ned;
						const Vector3f pos_ctl_course_direction_local = _vehicle_attitude_quaternion.rotateVector(velocity.normalized());
						_pos_ctl_course_direction = Vector2f(pos_ctl_course_direction_local(0), pos_ctl_course_direction_local(1));

					}

					// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
					const float vector_scaling = sqrtf(powf(_param_pp_lookahd_max.get(),
										2) + powf(_posctl_pure_pursuit.getCrosstrackError(), 2)) + _posctl_pure_pursuit.getDistanceOnLineSegment();
					const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + vector_scaling * _pos_ctl_course_direction;
					const float desired_heading = _posctl_pure_pursuit.calcDesiredHeading(target_waypoint_ned, _pos_ctl_start_position_ned,
								      _curr_pos_ned, desired_velocity_magnitude);
					const float heading_error = matrix::wrap_pi(desired_heading - _vehicle_yaw);
					const Vector2f desired_velocity = desired_velocity_magnitude * Vector2f(cosf(heading_error), sinf(heading_error));
					rover_mecanum_setpoint.forward_speed_setpoint = desired_velocity(0);
					rover_mecanum_setpoint.lateral_speed_setpoint = desired_velocity(1);
					rover_mecanum_setpoint.yaw_setpoint = _desired_yaw;
					rover_mecanum_setpoint.yaw_rate_setpoint = NAN;
				}

				_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);
			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_rover_mecanum_guidance.computeGuidance(_vehicle_yaw, _nav_state);
		break;

	default: // Unimplemented nav states will stop the rover
		rover_mecanum_setpoint_s rover_mecanum_setpoint{};
		rover_mecanum_setpoint.timestamp = timestamp;
		rover_mecanum_setpoint.forward_speed_setpoint = NAN;
		rover_mecanum_setpoint.forward_speed_setpoint_normalized = 0.f;
		rover_mecanum_setpoint.lateral_speed_setpoint = NAN;
		rover_mecanum_setpoint.lateral_speed_setpoint_normalized = 0.f;
		rover_mecanum_setpoint.yaw_rate_setpoint = NAN;
		rover_mecanum_setpoint.speed_diff_setpoint_normalized = 0.f;
		rover_mecanum_setpoint.yaw_setpoint = NAN;
		_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);
		break;
	}

	if (!_armed) { // Reset when disarmed
		_rover_mecanum_control.resetControllers();
		_yaw_ctl = false;
	}

	_rover_mecanum_control.computeMotorCommands(_vehicle_yaw, _vehicle_yaw_rate, _vehicle_forward_speed,
			_vehicle_lateral_speed);

}

void RoverMecanum::updateSubscriptions()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.nav_state != _nav_state) {
			_rover_mecanum_control.resetControllers();
			_yaw_ctl = false;

			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
			    || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
				_rover_mecanum_guidance.setDesiredYaw(_vehicle_yaw);
			}
		}

		_nav_state = vehicle_status.nav_state;
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		// Apply threshold to the velocity measurement to cut off measurement noise when standing still
		_vehicle_forward_speed = fabsf(velocity_in_body_frame(0)) > SPEED_THRESHOLD ? velocity_in_body_frame(0) : 0.f;
		_vehicle_lateral_speed = fabsf(velocity_in_body_frame(1)) > SPEED_THRESHOLD ? velocity_in_body_frame(1) : 0.f;
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);

		// Apply threshold to the yaw rate measurement if the rover is standing still to avoid stuttering due to closed loop yaw(rate) control
		if ((fabsf(_vehicle_forward_speed) > FLT_EPSILON && fabsf(_vehicle_lateral_speed) > FLT_EPSILON)
		    || fabsf(vehicle_angular_velocity.xyz[2]) > YAW_RATE_THRESHOLD) {
			_vehicle_yaw_rate = vehicle_angular_velocity.xyz[2];

		} else {
			_vehicle_yaw_rate = 0.f;
		}

	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

}

int RoverMecanum::task_spawn(int argc, char *argv[])
{
	RoverMecanum *instance = new RoverMecanum();

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

int RoverMecanum::custom_command(int argc, char *argv[])
{
	return print_usage("unk_timestampn command");
}

int RoverMecanum::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Mecanum controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_mecanum", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_mecanum_main(int argc, char *argv[])
{
	return RoverMecanum::main(argc, argv);
}
