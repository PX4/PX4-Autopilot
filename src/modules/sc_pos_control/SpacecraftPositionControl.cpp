/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "SpacecraftPositionControl.hpp"

#include <float.h>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

SpacecraftPositionControl::SpacecraftPositionControl() :
	SuperBlock(nullptr, "SPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
	parameters_update(true);
}

SpacecraftPositionControl::~SpacecraftPositionControl()
{
	perf_free(_cycle_perf);
}

bool SpacecraftPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void SpacecraftPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_vel_all.get() >= 0.f) {
			float all_vel = _param_mpc_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(all_vel);
			num_changed += _param_mpc_vel_cruise.commit_no_notification(all_vel);
			num_changed += _param_mpc_vel_max.commit_no_notification(all_vel);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		_control.setPositionGains(Vector3f(_param_mpc_pos_p.get(), _param_mpc_pos_p.get(), _param_mpc_pos_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_vel_p_acc.get(), _param_mpc_vel_p_acc.get(), _param_mpc_vel_p_acc.get()),
			Vector3f(_param_mpc_vel_i_acc.get(), _param_mpc_vel_i_acc.get(), _param_mpc_vel_i_acc.get()),
			Vector3f(_param_mpc_vel_d_acc.get(), _param_mpc_vel_d_acc.get(), _param_mpc_vel_d_acc.get()));

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_vel_cruise.get() > _param_mpc_vel_max.get()) {
			_param_mpc_vel_cruise.set(_param_mpc_vel_max.get());
			_param_mpc_vel_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>SPC_VEL_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("sc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>SPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("sc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_vel_max.get());
		}

		yaw_rate = math::radians(_param_mpc_man_y_max.get());
	}
}

PositionControlStates SpacecraftPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const vehicle_attitude_s &vehicle_attitude)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		states.velocity.xy() = velocity_xy;
		states.acceleration(0) = _vel_x_deriv.update(velocity_xy(0));
		states.acceleration(1) = _vel_y_deriv.update(velocity_xy(1));

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {
		states.velocity(2) = vehicle_local_position.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	if (PX4_ISFINITE(vehicle_attitude.q[0]) && PX4_ISFINITE(vehicle_attitude.q[1]) && PX4_ISFINITE(vehicle_attitude.q[2])
	    && PX4_ISFINITE(vehicle_attitude.q[3])) {
		states.quaternion = Quatf(vehicle_attitude.q);

	} else {
		states.quaternion = Quatf();
	}

	return states;
}

void SpacecraftPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;
	vehicle_attitude_s v_att;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		// set _dt in controllib Block for BlockDerivative
		setDt(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_control_position_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_control_position_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_control_position_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = ScPositionControl::empty_trajectory_setpoint;
				}
			}
		}

		// TODO: check if setpoint is different than the previous one and reset integral then
		// 		 _control.resetIntegral();
		_trajectory_setpoint_sub.update(&_setpoint);
		_vehicle_attitude_sub.update(&v_att);

		// adjust existing (or older) setpoint with any EKF reset deltas
		if ((_setpoint.timestamp != 0) && (_setpoint.timestamp < vehicle_local_position.timestamp)) {
			if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
				_setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
				_setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
			}

			if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
				_setpoint.velocity[2] += vehicle_local_position.delta_vz;
			}

			if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
				_setpoint.position[0] += vehicle_local_position.delta_xy[0];
				_setpoint.position[1] += vehicle_local_position.delta_xy[1];
			}

			if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
				_setpoint.position[2] += vehicle_local_position.delta_z;
			}

			if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
				_setpoint.yaw = wrap_pi(_setpoint.yaw + vehicle_local_position.delta_heading);
			}
		}

		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			_vel_x_deriv.reset();
			_vel_y_deriv.reset();
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			_vel_z_deriv.reset();
		}

		// save latest reset counters
		_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
		_vz_reset_counter = vehicle_local_position.vz_reset_counter;
		_xy_reset_counter = vehicle_local_position.xy_reset_counter;
		_z_reset_counter = vehicle_local_position.z_reset_counter;
		_heading_reset_counter = vehicle_local_position.heading_reset_counter;

		PositionControlStates states{set_vehicle_states(vehicle_local_position, v_att)};

		poll_manual_setpoint(dt, vehicle_local_position, v_att);

		if (_vehicle_control_mode.flag_control_position_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {
				PX4_INFO("Setpoint time: %f, Vehicle local pos time: %f, Pos Control Enabled time: %f",
					 (double)_setpoint.timestamp, (double)vehicle_local_position.timestamp_sample,
					 (double)_time_position_control_enabled);
				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_control_position_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			_control.setThrustLimit(_param_mpc_thr_max.get());

			_control.setVelocityLimits(_param_mpc_vel_max.get());

			_control.setInputSetpoint(_setpoint);

			_control.setState(states);

			// Run position control
			if (!_control.update(dt)) {
				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
				_control.setVelocityLimits(_param_mpc_vel_max.get());
				_control.update(dt);
			}

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint, v_att);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		}
	}

	perf_end(_cycle_perf);
}

void SpacecraftPositionControl::poll_manual_setpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position,
		const vehicle_attitude_s &_vehicle_att)
{
	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_armed) {
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vehicle_control_mode.flag_control_climb_rate_enabled &&
			    !_vehicle_control_mode.flag_control_offboard_enabled) {

				if (_vehicle_control_mode.flag_control_attitude_enabled) {
					// We are in Stabilized mode
					// Generate position setpoints
					if (!stabilized_pos_sp_initialized) {
						// Initialize position setpoint
						target_pos_sp = Vector3f(vehicle_local_position.x, vehicle_local_position.y,
									 vehicle_local_position.z);

						const float vehicle_yaw = Eulerf(Quatf(_vehicle_att.q)).psi();
						_manual_yaw_sp = vehicle_yaw;
						stabilized_pos_sp_initialized = true;
					}

					// Update velocity setpoint
					Vector3f target_vel_sp = Vector3f(_manual_control_setpoint.pitch, _manual_control_setpoint.roll, 0.0);
					// TODO(@Pedro-Roque): probably need to move velocity to inertial frame
					target_pos_sp = target_pos_sp + target_vel_sp * dt;

					// Update _setpoint
					_setpoint.position[0] = target_pos_sp(0);
					_setpoint.position[1] = target_pos_sp(1);
					_setpoint.position[2] = target_pos_sp(2);

					_setpoint.velocity[0] = target_vel_sp(0);
					_setpoint.velocity[1] = target_vel_sp(1);
					_setpoint.velocity[2] = target_vel_sp(2);

					// Generate attitude setpoints
					float yaw_sp_move_rate = 0.0;

					if (_manual_control_setpoint.throttle > -0.9f) {
						yaw_sp_move_rate = _manual_control_setpoint.yaw * yaw_rate;
					}

					_manual_yaw_sp = wrap_pi(_manual_yaw_sp + yaw_sp_move_rate * dt);
					const float roll_body = 0.0;
					const float pitch_body = 0.0;

					Quatf q_sp(Eulerf(roll_body, pitch_body, _manual_yaw_sp));
					q_sp.copyTo(_setpoint.attitude);

					_setpoint.timestamp = hrt_absolute_time();

				} else {
					// We are in Manual mode
					stabilized_pos_sp_initialized = false;
				}

			} else {
				stabilized_pos_sp_initialized = false;
			}

			_manual_setpoint_last_called = hrt_absolute_time();
		}
	}
}

trajectory_setpoint_s SpacecraftPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = ScPositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = failsafe_setpoint.velocity[2] = 0.f;

	if (warn) {
		PX4_WARN("Failsafe: stop and wait");
	}

	return failsafe_setpoint;
}

int SpacecraftPositionControl::task_spawn(int argc, char *argv[])
{
	SpacecraftPositionControl *instance = new SpacecraftPositionControl();

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

int SpacecraftPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SpacecraftPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
	The controller has two loops: a P loop for position error and a PID loop for velocity error.
	Output of the velocity controller is thrust vector in the body frame, and the same target attitude
	received on the trajectory setpoint as quaternion.

	The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
	logging.
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sc_pos_control_main(int argc, char *argv[])
{
	return SpacecraftPositionControl::main(argc, argv);
}
