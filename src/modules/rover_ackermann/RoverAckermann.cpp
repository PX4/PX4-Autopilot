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

#include "RoverAckermann.hpp"

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_rover_ackermann_setpoint_pub.advertise();
	_ax_filter.setAlpha(0.05);
	_ay_filter.setAlpha(0.05);
	_az_filter.setAlpha(0.05);
	updateParams();
}

bool RoverAckermann::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();
}

void RoverAckermann::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateSubscriptions();

	// Generate and publish speed and steering setpoints
	hrt_abstime timestamp = hrt_absolute_time();

	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_ackermann_setpoint_s rover_ackermann_setpoint{};
				rover_ackermann_setpoint.timestamp =  timestamp;
				rover_ackermann_setpoint.forward_speed_setpoint =  NAN;
				rover_ackermann_setpoint.forward_speed_setpoint_normalized =  manual_control_setpoint.throttle;
				rover_ackermann_setpoint.steering_setpoint = NAN;
				rover_ackermann_setpoint.steering_setpoint_normalized = manual_control_setpoint.roll;
				rover_ackermann_setpoint.lateral_acceleration_setpoint = NAN;
				_rover_ackermann_setpoint_pub.publish(rover_ackermann_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_ackermann_setpoint_s rover_ackermann_setpoint{};
				rover_ackermann_setpoint.timestamp =  timestamp;
				rover_ackermann_setpoint.forward_speed_setpoint =  NAN;
				rover_ackermann_setpoint.forward_speed_setpoint_normalized =  manual_control_setpoint.throttle;
				rover_ackermann_setpoint.steering_setpoint = NAN;
				rover_ackermann_setpoint.steering_setpoint_normalized = NAN;
				rover_ackermann_setpoint.lateral_acceleration_setpoint = math::interpolate(manual_control_setpoint.roll, -1.f, 1.f,
						-_param_ra_max_lat_accel.get(), _param_ra_max_lat_accel.get());
				_rover_ackermann_setpoint_pub.publish(rover_ackermann_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				rover_ackermann_setpoint_s rover_ackermann_setpoint{};
				rover_ackermann_setpoint.timestamp = timestamp;
				rover_ackermann_setpoint.forward_speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
						-1.f, 1.f, -_param_ra_max_speed.get(), _param_ra_max_speed.get());
				rover_ackermann_setpoint.forward_speed_setpoint_normalized = NAN;
				rover_ackermann_setpoint.steering_setpoint = NAN;
				rover_ackermann_setpoint.steering_setpoint_normalized = NAN;
				rover_ackermann_setpoint.lateral_acceleration_setpoint = math::interpolate(math::deadzone(manual_control_setpoint.roll,
						STICK_DEADZONE), -1.f, 1.f, -_param_ra_max_lat_accel.get(), _param_ra_max_lat_accel.get());

				if (fabsf(rover_ackermann_setpoint.lateral_acceleration_setpoint) > FLT_EPSILON
				    || fabsf(rover_ackermann_setpoint.forward_speed_setpoint) < FLT_EPSILON) { // Closed loop yaw rate control
					_course_control = false;

				} else { // Course control if the steering input is zero (keep driving on a straight line)
					if (!_course_control) {
						_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
						_pos_ctl_start_position_ned = _curr_pos_ned;
						_course_control = true;
					}

					// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
					const float vector_scaling = sqrtf(powf(_param_pp_lookahd_max.get(),
										2) + powf(_posctl_pure_pursuit.getCrosstrackError(), 2)) + _posctl_pure_pursuit.getDistanceOnLineSegment();
					const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(
							rover_ackermann_setpoint.forward_speed_setpoint) *
									     vector_scaling * _pos_ctl_course_direction;
					// Calculate steering setpoint
					const float steering_setpoint = _ackermann_guidance.calcDesiredSteering(_posctl_pure_pursuit,
									target_waypoint_ned, _pos_ctl_start_position_ned, _curr_pos_ned, _param_ra_wheel_base.get(),
									rover_ackermann_setpoint.forward_speed_setpoint, _vehicle_yaw, _param_ra_max_steer_angle.get(), _armed);
					rover_ackermann_setpoint.lateral_acceleration_setpoint = powf(_vehicle_forward_speed,
							2.f) * tanf(steering_setpoint) / _param_ra_wheel_base.get();
				}

				_rover_ackermann_setpoint_pub.publish(rover_ackermann_setpoint);
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_ackermann_guidance.computeGuidance(_vehicle_forward_speed, _vehicle_yaw, _nav_state, _armed);
		break;

	default: // Unimplemented nav states will stop the rover
		rover_ackermann_setpoint_s rover_ackermann_setpoint{};
		rover_ackermann_setpoint.timestamp =  timestamp;
		rover_ackermann_setpoint.forward_speed_setpoint =  NAN;
		rover_ackermann_setpoint.forward_speed_setpoint_normalized =  0.f;
		rover_ackermann_setpoint.steering_setpoint = NAN;
		rover_ackermann_setpoint.steering_setpoint_normalized = 0.f;
		rover_ackermann_setpoint.lateral_acceleration_setpoint = NAN;
		_rover_ackermann_setpoint_pub.publish(rover_ackermann_setpoint);
		break;
	}

	if (!_armed) {
		_ackermann_control.resetControllers();
	}

	_ackermann_control.computeMotorCommands(_vehicle_forward_speed, _vehicle_yaw, _vehicle_lateral_acceleration);

}

void RoverAckermann::updateSubscriptions()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.nav_state != _nav_state) { // Reset on mode change
			_ackermann_control.resetControllers();
			_course_control = false;
		}

		_nav_state = vehicle_status.nav_state;
		_armed = vehicle_status.arming_state == 2;
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

		if (PX4_ISFINITE(vehicle_local_position.ax)) {
			_ax_filter.update(vehicle_local_position.ax);
		}

		if (PX4_ISFINITE(vehicle_local_position.ay)) {
			_ay_filter.update(vehicle_local_position.ay);
		}

		if (PX4_ISFINITE(vehicle_local_position.az)) {
			_az_filter.update(vehicle_local_position.az);
		}

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_forward_speed = fabsf(velocity_in_body_frame(0)) > SPEED_THRESHOLD ? velocity_in_body_frame(0) : 0.f;
		Vector3f acceleration_in_local_frame(_ax_filter.getState(), _ay_filter.getState(), _az_filter.getState());
		Vector3f acceleration_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(acceleration_in_local_frame);
		_vehicle_lateral_acceleration = acceleration_in_body_frame(1);
	}
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

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

int RoverAckermann::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAckermann::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover ackermann module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_ackermann", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_ackermann_main(int argc, char *argv[])
{
	return RoverAckermann::main(argc, argv);
}
