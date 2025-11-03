/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file MulticopterINDIRateControl.cpp
 *
 * Multicopter INDI rate control
 *
 * @author Rohan Inamdar <rninamdar@wpi.edu>
 */

#include "MulticopterINDIRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterINDIRateControl::MulticopterINDIRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_rotors(this),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterINDIRateControl::~MulticopterINDIRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterINDIRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_rotors.initializeEffectivenessMatrix(_actuator_effectiveness_config,
			EffectivenessUpdateReason::CONFIGURATION_UPDATE)) {
		PX4_ERR("Failed to initialize INDI effectiveness matrix");
		return false;
	}

	// Get the number of actuators from the configuration
	_num_actuators = _actuator_effectiveness_config.num_actuators_matrix[_actuator_effectiveness_config.selected_matrix];
	PX4_INFO("INDI: Initialized with %d actuators", _num_actuators);

	return true;
}

void
MulticopterINDIRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(0, 0, 0)), //disable integral control (set to Ki = 0 to not effect the rate control library)
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());

	//FIXME: find the delta t correctly bruh
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
MulticopterINDIRateControl::Run()
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

	Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> filtered_radps_vec;
	Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> filtered_radps_vec_dot;

	esc_status_s esc_status;

	if (_esc_status_sub.update(&esc_status)) {
		for (int i = 0; i < _num_actuators; i++) {
			_prev_esc_rad_per_sec_filtered(i) = filtered_radps_vec(i);
			_prev_esc_rad_per_sec_filtered_dot(i) = filtered_radps_vec_dot(i);
			float rad_per_sec = (float)(esc_status.esc[i].esc_rpm) * 2.f * M_PI_F / 60.f;
			filtered_radps_vec(i) = _radps_lpf[i].apply(rad_per_sec);
			const float dt = math::constrain(((esc_status.timestamp - _last_esc_status_update) * 1e-6f), 0.000125f, 0.02f);
			filtered_radps_vec_dot(i) = (filtered_radps_vec(i) - _prev_esc_rad_per_sec_filtered(i)) / dt;

		}

		_last_esc_status_update = esc_status.timestamp;
	}


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

		if (_param_mc_indi_adapt_en.get()) {

			matrix::Vector3f angular_accel_delta = angular_accel - _prev_angular_accel;
			matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> delta_radps_vec = filtered_radps_vec -
					_prev_esc_rad_per_sec_filtered;
			matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> delta_radps_vec_dot = filtered_radps_vec_dot -
					_prev_esc_rad_per_sec_filtered_dot;
			_rotors.adaptEffectivenessMatrix(_actuator_effectiveness_config, delta_radps_vec, delta_radps_vec_dot,
							 angular_accel_delta);

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

			Matrix<float, 3, ActuatorEffectiveness::NUM_ACTUATORS> G1 = _rotors.getG1(_actuator_effectiveness_config);
			Matrix<float, 3, ActuatorEffectiveness::NUM_ACTUATORS> G2 = _rotors.getG2(_actuator_effectiveness_config);
			Vector3f indi_torque_setpoint = computeIndiTorqueSetpoint(filtered_radps_vec, _prev_esc_rad_per_sec_filtered, dt, G1,
							G2);

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);


			_vehicle_torque_setpoint.pid_xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			_vehicle_torque_setpoint.pid_xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			_vehicle_torque_setpoint.pid_xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			Vector3f measured_body_torque = _Iv * angular_accel;


			// NOTE: to confirm tuning, the indi_torque_xyz should be the same as the measured_body_torque_xyz when graphed in plotjuggler
			_vehicle_torque_setpoint.indi_torque_xyz[0] = PX4_ISFINITE(indi_torque_setpoint(0)) ? indi_torque_setpoint(0) : 0.f;
			_vehicle_torque_setpoint.indi_torque_xyz[1] = PX4_ISFINITE(indi_torque_setpoint(1)) ? indi_torque_setpoint(1) : 0.f;
			_vehicle_torque_setpoint.indi_torque_xyz[2] = PX4_ISFINITE(indi_torque_setpoint(2)) ? indi_torque_setpoint(2) : 0.f;

			_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.pid_xyz[0] + _vehicle_torque_setpoint.indi_torque_xyz[0];
			_vehicle_torque_setpoint.xyz[1] = _vehicle_torque_setpoint.pid_xyz[1] + _vehicle_torque_setpoint.indi_torque_xyz[1];
			_vehicle_torque_setpoint.xyz[2] = _vehicle_torque_setpoint.pid_xyz[2] + _vehicle_torque_setpoint.indi_torque_xyz[2];

			_vehicle_torque_setpoint.measured_body_torque_xyz[0] = PX4_ISFINITE(measured_body_torque(0)) ? measured_body_torque(
						0) : 0.f;
			_vehicle_torque_setpoint.measured_body_torque_xyz[1] = PX4_ISFINITE(measured_body_torque(1)) ? measured_body_torque(
						1) : 0.f;
			_vehicle_torque_setpoint.measured_body_torque_xyz[2] = PX4_ISFINITE(measured_body_torque(2)) ? measured_body_torque(
						2) : 0.f;

			_prev_esc_rad_per_sec_filtered = filtered_radps_vec;
			_prev_angular_accel = angular_accel;


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
						_vehicle_torque_setpoint.xyz[i] = math::constrain(_vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);

			updateActuatorControlsStatus(_vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

Vector3f MulticopterINDIRateControl::computeIndiTorqueSetpoint(const Vector<float, ActuatorEffectiveness::NUM_ACTUATORS>
		&filtered_radps_vec,
		const Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> &prev_esc_rad_per_sec_filtered,
		float dt,
		const Matrix<float, 3, ActuatorEffectiveness::NUM_ACTUATORS> &G1,
		const Matrix<float, 3, ActuatorEffectiveness::NUM_ACTUATORS> &G2)
{
	Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> radps_vec_squared = filtered_radps_vec.emult(filtered_radps_vec);
	Vector3f G1_term = (G1 * (radps_vec_squared));
	_vehicle_torque_setpoint.g1_term[0] = G1_term(0);
	_vehicle_torque_setpoint.g1_term[1] = G1_term(1);
	_vehicle_torque_setpoint.g1_term[2] = G1_term(2);
	Vector3f G2_term = (G2 * (filtered_radps_vec - prev_esc_rad_per_sec_filtered)) / dt;
	_vehicle_torque_setpoint.g2_term[0] = G2_term(0);
	_vehicle_torque_setpoint.g2_term[1] = G2_term(1);
	_vehicle_torque_setpoint.g2_term[2] = G2_term(2);
	Vector3f filtered_body_torque_setpoint = G1_term + G2_term;

	return filtered_body_torque_setpoint;
}

matrix::Vector3f MulticopterINDIRateControl::computeDesiredAngularAcceleration(
	const matrix::Quatf &q_current,
	const matrix::Quatf &q_desired,
	const matrix::Vector3f &angular_vel_body,
	const matrix::Vector3f &angular_vel_desired,
	const matrix::Vector3f &angular_accel_body
) {
	//Computing error quaternion (This is basicallty the best way)
	matrix::Quatf q_error = q_current.inversed() * q_desired;

	//Extract the attitude component of error
	float sign_qw = (q_error(0) >= 0.0f) ? 1.0f : -1.0f;
	matrix::Vector3f q_e_red(
		2.0f * sign_qw * q_error(1),
        	2.0f * sign_qw * q_error(2),
        	2.0f * sign_qw * q_error(3)
    );
    	// take yaw out
    	float q_e_yaw_w = q_e_red(2);
    	float sign_yaw = (q_e_yaw_w >= 0.0f) ? 1.0f : -1.0f;

    	// get angular vel error
    	matrix::Vector3f omega_error = angular_vel_desired - angular_vel_body;

    	// Build gain vector
    	matrix::Vector3f K_omega(
        	_param_k_omega_r.get(),
        	_param_k_omega_p.get(),
        	_param_k_omega_y.get()
    	);

	// Final equation

    	matrix::Vector3f alpha_desired =
        	_param_k_q_red.get() * q_e_red +
        	_param_k_e_yaw.get() * sign_yaw * matrix::Vector3f(0.0f, 0.0f, q_e_yaw_w) +
        	K_omega.emult(omega_error) +
        	angular_accel_body;

    	return alpha_desired;
}

void MulticopterINDIRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
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

int MulticopterINDIRateControl::task_spawn(int argc, char *argv[])
{
	MulticopterINDIRateControl *instance = new MulticopterINDIRateControl();

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

int MulticopterINDIRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterINDIRateControl::print_usage(const char *reason)
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

extern "C" __EXPORT int mc_indi_rate_control_main(int argc, char *argv[])
{
	return MulticopterINDIRateControl::main(argc, argv);
}
