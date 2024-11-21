/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverDifferential.hpp"

RoverDifferential::RoverDifferential() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	_rover_differential_setpoint_pub.advertise();
}

bool RoverDifferential::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverDifferential::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_rd_max_yaw_rate.get() * M_DEG_TO_RAD_F;
}

void RoverDifferential::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();

	// Generate and publish attitude, rate and speed setpoints
	hrt_abstime timestamp = hrt_absolute_time();

	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = NAN;
				rover_differential_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_differential_setpoint.yaw_setpoint = NAN;

				if (_max_yaw_rate > FLT_EPSILON && _param_rd_max_thr_yaw_r.get() > FLT_EPSILON) {
					const float scaled_yaw_rate_input = math::interpolate<float>(manual_control_setpoint.roll,
									    -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
					const float speed_diff = scaled_yaw_rate_input * _param_rd_wheel_track.get() / 2.f;
					rover_differential_setpoint.speed_diff_setpoint_normalized = math::interpolate<float>(speed_diff,
							-_param_rd_max_thr_yaw_r.get(), _param_rd_max_thr_yaw_r.get(), -1.f, 1.f);

				} else {
					rover_differential_setpoint.speed_diff_setpoint_normalized = manual_control_setpoint.roll;

				}

				rover_differential_setpoint.yaw_rate_setpoint = NAN;
				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}
		} break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = NAN;
				rover_differential_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_differential_setpoint.yaw_rate_setpoint = math::interpolate<float>(manual_control_setpoint.roll,
						-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_differential_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_differential_setpoint.yaw_setpoint = NAN;
				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_STAB: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = NAN;
				rover_differential_setpoint.forward_speed_setpoint_normalized = manual_control_setpoint.throttle;
				rover_differential_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
						STICK_DEADZONE), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_differential_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_differential_setpoint.yaw_setpoint = NAN;

				if (fabsf(rover_differential_setpoint.yaw_rate_setpoint) > FLT_EPSILON
				    || fabsf(rover_differential_setpoint.forward_speed_setpoint_normalized) < FLT_EPSILON) { // Closed loop yaw rate control
					_yaw_ctl = false;


				} else { // Closed loop yaw control if the yaw rate input is zero (keep current yaw)
					if (!_yaw_ctl) {
						_stab_desired_yaw = _vehicle_yaw;
						_yaw_ctl = true;
					}

					rover_differential_setpoint.yaw_setpoint = _stab_desired_yaw;
					rover_differential_setpoint.yaw_rate_setpoint = NAN;

				}

				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_differential_setpoint_s rover_differential_setpoint{};
				rover_differential_setpoint.timestamp = timestamp;
				rover_differential_setpoint.forward_speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
						-1.f, 1.f, -_param_rd_max_speed.get(), _param_rd_max_speed.get());
				rover_differential_setpoint.forward_speed_setpoint_normalized = NAN;
				rover_differential_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
						STICK_DEADZONE), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
				rover_differential_setpoint.speed_diff_setpoint_normalized = NAN;
				rover_differential_setpoint.yaw_setpoint = NAN;

				if (fabsf(rover_differential_setpoint.yaw_rate_setpoint) > FLT_EPSILON
				    || fabsf(rover_differential_setpoint.forward_speed_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
					_yaw_ctl = false;


				} else { // Course control if the yaw rate input is zero (keep driving on a straight line)
					if (!_yaw_ctl) {
						_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
						_pos_ctl_start_position_ned = _curr_pos_ned;
						_yaw_ctl = true;
					}

					// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
					const float vector_scaling = sqrtf(powf(_param_pp_lookahd_max.get(),
										2) + powf(_posctl_pure_pursuit.getCrosstrackError(), 2)) + _posctl_pure_pursuit.getDistanceOnLineSegment();
					const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(
							rover_differential_setpoint.forward_speed_setpoint) *
									     vector_scaling * _pos_ctl_course_direction;
					// Calculate yaw setpoint
					const float yaw_setpoint = _posctl_pure_pursuit.calcDesiredHeading(target_waypoint_ned,
								   _pos_ctl_start_position_ned, _curr_pos_ned, fabsf(_vehicle_forward_speed));
					rover_differential_setpoint.yaw_setpoint = sign(rover_differential_setpoint.forward_speed_setpoint) >= 0 ?
							yaw_setpoint : matrix::wrap_pi(M_PI_F + yaw_setpoint); // Flip yaw setpoint when driving backwards
					rover_differential_setpoint.yaw_rate_setpoint = NAN;

				}

				_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_rover_differential_guidance.computeGuidance(_vehicle_yaw, _vehicle_forward_speed, _nav_state);
		break;

	default: // Unimplemented nav states will stop the rover
		_rover_differential_control.resetControllers();
		_yaw_ctl = false;
		rover_differential_setpoint_s rover_differential_setpoint{};
		rover_differential_setpoint.forward_speed_setpoint = NAN;
		rover_differential_setpoint.forward_speed_setpoint_normalized = 0.f;
		rover_differential_setpoint.yaw_rate_setpoint = NAN;
		rover_differential_setpoint.speed_diff_setpoint_normalized = 0.f;
		rover_differential_setpoint.yaw_setpoint = NAN;
		_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
		break;
	}

	if (!_armed) { // Reset when disarmed
		_rover_differential_control.resetControllers();
		_yaw_ctl = false;
	}

	_rover_differential_control.computeMotorCommands(_vehicle_yaw, _vehicle_yaw_rate, _vehicle_forward_speed);

}

void RoverDifferential::updateSubscriptions()
{

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.nav_state != _nav_state) { // Reset on mode change
			_rover_differential_control.resetControllers();
			_yaw_ctl = false;
		}

		_nav_state = vehicle_status.nav_state;
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > YAW_RATE_THRESHOLD ? vehicle_angular_velocity.xyz[2] : 0.f;
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_forward_speed = fabsf(velocity_in_body_frame(0)) > SPEED_THRESHOLD ? velocity_in_body_frame(0) : 0.f;
	}
}

int RoverDifferential::task_spawn(int argc, char *argv[])
{
	RoverDifferential *instance = new RoverDifferential();

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

int RoverDifferential::custom_command(int argc, char *argv[])
{
	return print_usage("unk_timestampn command");
}

int RoverDifferential::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Differential controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_differential", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_differential_main(int argc, char *argv[])
{
	return RoverDifferential::main(argc, argv);
}
