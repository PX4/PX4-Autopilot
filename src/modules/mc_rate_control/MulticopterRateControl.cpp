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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());

	// INDI
	_G1_K = diag(Vector3f(_param_mc_indi_g1_roll.get(), _param_mc_indi_g1_pitch.get(), _param_mc_indi_g1_yaw.get()));
	_G2_K = diag(Vector3f(_param_mc_indi_g2_roll.get(), _param_mc_indi_g2_pitch.get(), _param_mc_indi_g2_yaw.get()));

	if (_param_imu_gyro_cutoff.get() > 0.f) {
		// The perf_mean() function returns the average elapsed time of the counter in seconds.
		const float avg_interval_s = perf_mean(_loop_perf) > 0.f ? perf_mean(_loop_perf) : 0.001f; //default to 1 kHz

		if (avg_interval_s > 0.f) { // Avoid division by zero if the interval is not yet measured
			const float sample_rate_hz = 1.f / avg_interval_s;

			for (auto &radps_lpf : _radps_lpf) {
				radps_lpf.set_cutoff_frequency(sample_rate_hz, _param_imu_gyro_cutoff.get());
			}
		}
	}
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	_esc_status_sub.update(&_esc_status);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> filtered_radps_vec;

		if (_vehicle_control_mode.flag_control_rates_indi_enabled) {

			if (_actuator_effectiveness_matrix_sub.updated()) {
				if (_actuator_effectiveness_matrix_sub.copy(&_actuator_effectiveness_matrix)) {
					_num_actuators = _actuator_effectiveness_matrix.num_actuators;

					// seperate the row major matrix into the torque (G1) and thrust (G2) matrices
					// NOTE: the G1, G2 constant gains, K, are necessary as the PID approximates Iv, and such the output of the PID rate controller
					// *counts* as torque instead of angular acceleration.setpoints, and thus is used against the INDI computed torque_filtered
					// as the original equation of torque_setpoint = torque_filtered + Iv * (ang_accel_setpoint - Omega_dot_filtered_gyro) is approximated to: torque_setpoint = torque_filtered * K + torque_output_from_PID
					// in an NMPC approach this would never be needed, but the PID is pretty robust here that its a good approximation
					for (int i = 0; i < 3; i++) {

						for (int j = 0; j < _num_actuators; j++) {
							_G1(i, j) = _actuator_effectiveness_matrix.effectiveness_matrix_row_major[i * 16 + j]; //row has 16 columns (even if they are empty/not have 16 actuators)
						}
					}

					for (int i = 0; i < 3; i++) {
						for (int j = 0; j < _num_actuators; j++) {
							_G2(i, j) = _actuator_effectiveness_matrix.effectiveness_matrix_row_major[(i + 3) * 16 + j]; //thrust comes 3 rows after torque, so offset by 3
						}
					}
				}
			}

			if (_esc_status.timestamp > 0) { //check if atleast one esc status is available
				for (int i = 0; i < _num_actuators; i++) {
					float rad_per_sec = (float)(_esc_status.esc[i].esc_rpm) * 2.f * M_PI_F / 60.f;
					filtered_radps_vec(i) = _radps_lpf[i].apply(rad_per_sec);
				}
			}
			else {
				filtered_radps_vec.zero();
				PX4_WARN("No ESC status available for INDI");
			}
		}

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
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

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			vehicle_torque_setpoint.delta_xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.delta_xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.delta_xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;
			//PX4_INFO("torque_setpoint: %f, %f, %f", (double)torque_setpoint(0), (double)torque_setpoint(1), (double)torque_setpoint(2));

			// TODO: a possible optimization is to manually multiply the G1 and G2 matrices with the radps_vec_squared and filtered_radps_vec - _prev_esc_rad_per_sec_filtered vectors
			// as this would avoid the need to multiply by the full 16 vector, which is full of zeros for most cases (quadrotors)
			if (_vehicle_control_mode.flag_control_rates_indi_enabled) {
				matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> radps_vec_squared = filtered_radps_vec.emult(filtered_radps_vec);
				//PX4_INFO("radps_vec_squared: %f, %f, %f", (double)radps_vec_squared(0), (double)radps_vec_squared(1), (double)radps_vec_squared(2));
				//PX4_INFO("filtered_radps_vec: %f, %f, %f", (double)filtered_radps_vec(0), (double)filtered_radps_vec(1), (double)filtered_radps_vec(2));
				Vector3f G1_term = _G1_K * (_G1 * (radps_vec_squared));
				Vector3f G2_term = _G2_K * (_G2 * (filtered_radps_vec - _prev_esc_rad_per_sec_filtered)) / dt;
				Vector3f filtered_body_torque_setpoint = G1_term + G2_term;
				Vector3f filtered_body_torque_setpoint_no_constant = _G1 * (radps_vec_squared) + _G2 * (filtered_radps_vec - _prev_esc_rad_per_sec_filtered) / dt;
				Vector3f error = filtered_body_torque_setpoint_no_constant - _Iv * angular_accel;
				//PX4_INFO("G1 term: %f, %f, %f", (double)G1_term(0), (double)G1_term(1), (double)G1_term(2));
				//PX4_INFO("G2 term: %f, %f, %f", (double)G2_term(0), (double)G2_term(1), (double)G2_term(2));
				vehicle_torque_setpoint.xyz[0] += PX4_ISFINITE(filtered_body_torque_setpoint(0)) ? filtered_body_torque_setpoint(0) : 0.f;
				vehicle_torque_setpoint.xyz[1] += PX4_ISFINITE(filtered_body_torque_setpoint(1)) ? filtered_body_torque_setpoint(1) : 0.f;
				vehicle_torque_setpoint.xyz[2] += PX4_ISFINITE(filtered_body_torque_setpoint(2)) ? filtered_body_torque_setpoint(2) : 0.f;

				vehicle_torque_setpoint.g1_term[0] = PX4_ISFINITE(G1_term(0)) ? G1_term(0) : 0.f;
				vehicle_torque_setpoint.g1_term[1] = PX4_ISFINITE(G1_term(1)) ? G1_term(1) : 0.f;
				vehicle_torque_setpoint.g1_term[2] = PX4_ISFINITE(G1_term(2)) ? G1_term(2) : 0.f;
				vehicle_torque_setpoint.g2_term[0] = PX4_ISFINITE(G2_term(0)) ? G2_term(0) : 0.f;
				vehicle_torque_setpoint.g2_term[1] = PX4_ISFINITE(G2_term(1)) ? G2_term(1) : 0.f;
				vehicle_torque_setpoint.g2_term[2] = PX4_ISFINITE(G2_term(2)) ? G2_term(2) : 0.f;

				vehicle_torque_setpoint.filtered_xyz[0] = PX4_ISFINITE(filtered_body_torque_setpoint(0)) ? filtered_body_torque_setpoint(0) : 0.f;
				vehicle_torque_setpoint.filtered_xyz[1] = PX4_ISFINITE(filtered_body_torque_setpoint(1)) ? filtered_body_torque_setpoint(1) : 0.f;
				vehicle_torque_setpoint.filtered_xyz[2] = PX4_ISFINITE(filtered_body_torque_setpoint(2)) ? filtered_body_torque_setpoint(2) : 0.f;

				vehicle_torque_setpoint.filtered_xyz_noconst[0] = PX4_ISFINITE(filtered_body_torque_setpoint_no_constant(0)) ? filtered_body_torque_setpoint_no_constant(0) : 0.f;
				vehicle_torque_setpoint.filtered_xyz_noconst[1] = PX4_ISFINITE(filtered_body_torque_setpoint_no_constant(1)) ? filtered_body_torque_setpoint_no_constant(1) : 0.f;
				vehicle_torque_setpoint.filtered_xyz_noconst[2] = PX4_ISFINITE(filtered_body_torque_setpoint_no_constant(2)) ? filtered_body_torque_setpoint_no_constant(2) : 0.f;

				vehicle_torque_setpoint.error[0] = PX4_ISFINITE(error(0)) ? error(0) : 0.f;
				vehicle_torque_setpoint.error[1] = PX4_ISFINITE(error(1)) ? error(1) : 0.f;
				vehicle_torque_setpoint.error[2] = PX4_ISFINITE(error(2)) ? error(2) : 0.f;

				_prev_esc_rad_per_sec_filtered = filtered_radps_vec;
			}

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
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

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
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

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

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

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
