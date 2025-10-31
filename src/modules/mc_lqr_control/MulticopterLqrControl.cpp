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

#include "MulticopterLqrControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;

MulticopterLqrControl::MulticopterLqrControl(bool vtol) :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	// WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	// _vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
	parameters_update(true);
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	reset_setpoint_to_nan(_setpoint);
	_takeoff_status_pub.advertise();
}

MulticopterLqrControl::~MulticopterLqrControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterLqrControl::init()
{
	// if (!_local_pos_sub.registerCallback()) {
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	// ScheduleNow();

	return true;
}

int MulticopterLqrControl::parameters_update(bool force)
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

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		// _control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		// _control.setVelocityGains(
		// 	Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
		// 	Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
		// 	Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		// _control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}

	return OK;
}

PositionControlStates MulticopterLqrControl::set_vehicle_states(const vehicle_local_position_s &local_pos, const vehicle_attitude_s &vehicle_attitude, const vehicle_angular_velocity_s &vehicle_angular_velocity)
{
	PositionControlStates states;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		states.position(0) = local_pos.x;
		states.position(1) = local_pos.y;

	} else {
		states.position(0) = NAN;
		states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;

	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		states.velocity(0) = local_pos.vx;
		states.velocity(1) = local_pos.vy;
		// states.acceleration(0) = _vel_x_deriv.update(local_pos.vx);
		// states.acceleration(1) = _vel_y_deriv.update(local_pos.vy);

	} else {
		states.velocity(0) = NAN;
		states.velocity(1) = NAN;
		// states.acceleration(0) = NAN;
		// states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		states.velocity(2) = local_pos.vz;
		// states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		// states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	Eulerf euler = Eulerf(Quatf(vehicle_attitude.q));
	states.attitude(0) = euler.phi();
	states.attitude(1) = euler.theta();
	states.attitude(2) = euler.psi();

	states.angular_velocity(0) = vehicle_angular_velocity.xyz[0];
	states.angular_velocity(1) = vehicle_angular_velocity.xyz[1];
	states.angular_velocity(2) = vehicle_angular_velocity.xyz[2];

	// states.yaw = local_pos.heading;

	return states;
}

void MulticopterLqrControl::Run()
{
	if (should_exit()) {
		// _local_pos_sub.unregisterCallback();
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	// ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s local_pos;
	vehicle_attitude_s vehicle_attitude;
	vehicle_angular_velocity_s vehicle_angular_velocity;

	if (_local_pos_sub.update(&local_pos)) {
		_local_pos = local_pos;
	} else {
		local_pos = _local_pos;
	}

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		_vehicle_attitude = vehicle_attitude;
	} else {
		vehicle_attitude = _vehicle_attitude;
	}

	// if (_local_pos_sub.update(&local_pos) && _vehicle_attitude_sub.update(&vehicle_attitude) && _vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		// set _dt in controllib Block for BlockDerivative
		setDt(dt);

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		PositionControlStates states{set_vehicle_states(local_pos, vehicle_attitude, vehicle_angular_velocity)};

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			const bool is_trajectory_setpoint_updated = _trajectory_setpoint_sub.update(&_setpoint);

			// adjust existing (or older) setpoint with any EKF reset deltas
			if (_setpoint.timestamp < local_pos.timestamp) {
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
					_setpoint.vx += local_pos.delta_vxy[0];
					_setpoint.vy += local_pos.delta_vxy[1];
				}

				if (local_pos.vz_reset_counter != _vz_reset_counter) {
					_setpoint.vz += local_pos.delta_vz;
				}

				if (local_pos.xy_reset_counter != _xy_reset_counter) {
					_setpoint.x += local_pos.delta_xy[0];
					_setpoint.y += local_pos.delta_xy[1];
				}

				if (local_pos.z_reset_counter != _z_reset_counter) {
					_setpoint.z += local_pos.delta_z;
				}

				if (local_pos.heading_reset_counter != _heading_reset_counter) {
					_setpoint.yaw += local_pos.delta_heading;
				}
			}

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				bool want_takeoff = _vehicle_control_mode.flag_armed && _vehicle_land_detected.landed
						    && hrt_elapsed_time(&_setpoint.timestamp) < 1_s;

				if (want_takeoff && PX4_ISFINITE(_setpoint.z)
				    && (_setpoint.z < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.vz)
					   && (_setpoint.vz < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, time_stamp_now);

			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);

			if (is_trajectory_setpoint_updated) {
				// make sure takeoff ramp is not amended by acceleration feed-forward
				if (!flying) {
					_setpoint.acceleration[2] = NAN;
					// hover_thrust maybe reset on takeoff
					_control.setHoverThrust(_param_mpc_thr_hover.get());
				}

				const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				if (not_taken_off || flying_but_ground_contact) {
					// we are not flying yet and need to avoid any corrections
					reset_setpoint_to_nan(_setpoint);
					Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

					// prevent any integrator windup
					// _control.resetIntegral();
				}
			}

			// // limit tilt during takeoff ramupup
			// const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
			// 			     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			// _control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			// const float speed_up = _takeoff.updateRamp(dt,
			// 		       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			// const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
			// 			 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;

			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			// _control.setVelocityLimits(
			// 	_param_mpc_xy_vel_max.get(),
			// 	math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
			// 	math::max(speed_down, 0.f));

			_control.setInputSetpoint(_setpoint);

			// update states
			if (!PX4_ISFINITE(_setpoint.z)
			    && PX4_ISFINITE(_setpoint.vz) && (fabsf(_setpoint.vz) > FLT_EPSILON)
			    && PX4_ISFINITE(local_pos.z_deriv) && local_pos.z_valid && local_pos.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.vz) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			_control.setState(states);

			// Run position control
			if (_control.update(dt)) {
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} else {
				// Failsafe
				if ((time_stamp_now - _last_warn) > 2_s) {
					PX4_WARN("invalid setpoints");
					_last_warn = time_stamp_now;
				}

				vehicle_local_position_setpoint_s failsafe_setpoint{};

				failsafe(time_stamp_now, failsafe_setpoint, states, !was_in_failsafe);

				// reset constraints
				_vehicle_constraints = {0, NAN, NAN, false, {}};

				_control.setInputSetpoint(failsafe_setpoint);
				// _control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
				_control.update(dt);
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// // Publish attitude setpoint output
			// vehicle_attitude_setpoint_s attitude_setpoint{};
			// _control.getAttitudeSetpoint(attitude_setpoint);
			// attitude_setpoint.timestamp = hrt_absolute_time();
			// _vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

			// Publish actuator controls
			actuator_controls_s actuator_controls{};
			_control.getActuatorControls(actuator_controls);
			actuator_controls.timestamp = hrt_absolute_time();
			_actuator_pub.publish(actuator_controls);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    time_stamp_now);
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}

		// save latest reset counters
		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
		_heading_reset_counter = local_pos.heading_reset_counter;
	}

	perf_end(_cycle_perf);
}

void MulticopterLqrControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
		const PositionControlStates &states, bool warn)
{
	// do not warn while we are disarmed, as we might not have valid setpoints yet
	if (!_vehicle_control_mode.flag_armed) {
		warn = false;
	}

	// Only react after a short delay
	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (_failsafe_land_hysteresis.get_state()) {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(states.velocity(0)) && PX4_ISFINITE(states.velocity(1))) {
			// don't move along xy
			setpoint.vx = setpoint.vy = 0.f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// descend with land speed since we can't stop
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_mpc_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(states.velocity(2))) {
			// don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

void MulticopterLqrControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

int MulticopterLqrControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterLqrControl *instance = new MulticopterLqrControl(vtol);

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

int MulticopterLqrControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterLqrControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_lqr_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_lqr_control_main(int argc, char *argv[])
{
	return MulticopterLqrControl::main(argc, argv);
}
