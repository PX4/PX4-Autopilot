/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "SpacecraftRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

SpacecraftRateControl::SpacecraftRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_controller_status_pub.advertise();
	updateParams();
}

void SpacecraftRateControl::updateParams()
{
	ModuleParams::updateParams();
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_sc_rollrate_k.get(), _param_sc_pitchrate_k.get(), _param_sc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_sc_rollrate_p.get(), _param_sc_pitchrate_p.get(), _param_sc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_sc_rollrate_i.get(), _param_sc_pitchrate_i.get(), _param_sc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_sc_rollrate_d.get(), _param_sc_pitchrate_d.get(), _param_sc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_sc_rr_int_lim.get(), _param_sc_pr_int_lim.get(), _param_sc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_sc_rollrate_ff.get(), _param_sc_pitchrate_ff.get(), _param_sc_yawrate_ff.get()));

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_sc_acro_r_max.get()), radians(_param_sc_acro_p_max.get()),
				  radians(_param_sc_acro_y_max.get()));
	_manual_force_max = _param_sc_manual_f_max.get();
	_manual_torque_max = _param_sc_manual_t_max.get();
}

void SpacecraftRateControl::updateRateControl()
{
	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled &&
		    !_vehicle_control_mode.flag_control_attitude_enabled) {
			// Here we can be in: Manual Mode or Acro Mode
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				if (_vehicle_control_mode.flag_control_rates_enabled) {
					// manual rates control - ACRO mode
					const Vector3f man_rate_sp{manual_control_setpoint.roll,
								   -manual_control_setpoint.pitch,
								   manual_control_setpoint.yaw};

					_rates_setpoint = man_rate_sp * 5;
					_thrust_setpoint(2) = -manual_control_setpoint.throttle;
					_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

					// publish rate setpoint
					vehicle_rates_setpoint.roll = _rates_setpoint(0);
					vehicle_rates_setpoint.pitch = _rates_setpoint(1);
					vehicle_rates_setpoint.yaw = _rates_setpoint(2);
					_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
					vehicle_rates_setpoint.timestamp = hrt_absolute_time();

					_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);

				} else if (!_vehicle_control_mode.flag_control_rates_enabled) {
					// Manual/direct control
					// Yaw stick commands rotational moment, Roll/Pitch stick commands translational forces
					// All other axis are set as zero (We only have four channels on the manual control inputs)
					_thrust_setpoint(0) = math::constrain((manual_control_setpoint.pitch * _manual_force_max), -1.f, 1.f);
					_thrust_setpoint(1) = math::constrain((manual_control_setpoint.roll * _manual_force_max), -1.f, 1.f);
					_thrust_setpoint(2) = 0.0;

					_torque_setpoint(0) = _torque_setpoint(1) = 0.0;
					_torque_setpoint(2) = math::constrain((manual_control_setpoint.yaw * _manual_torque_max), -1.f, 1.f);

					// publish thrust and torque setpoints
					vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
					vehicle_torque_setpoint_s vehicle_torque_setpoint{};

					_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
					_torque_setpoint.copyTo(vehicle_torque_setpoint.xyz);

					vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
					vehicle_thrust_setpoint.timestamp = hrt_absolute_time();

					vehicle_torque_setpoint.timestamp = hrt_absolute_time();
					vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;

					_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
					_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

					updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
				}
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			// Get rates from other controllers (e.g. position or attitude controller)
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll) ? vehicle_rates_setpoint.roll : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw) ? vehicle_rates_setpoint.yaw : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {
			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}
			}

			const Vector3f torque_sp = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, false);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_sp(0)) ? torque_sp(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_sp(1)) ? torque_sp(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_sp(2)) ? torque_sp(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_sc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] =
							math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] =
							math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		}
	}
}

void SpacecraftRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {
		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}
