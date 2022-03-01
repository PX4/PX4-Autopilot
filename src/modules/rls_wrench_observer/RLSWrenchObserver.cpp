/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file RLSWrenchObserver.cpp
 * @brief RLS parameter identification and external wrench observer
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#include "RLSWrenchObserver.hpp"

RLSWrenchObserver::RLSWrenchObserver() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_valid_hysteresis.set_hysteresis_time_from(false, 2_s);
	updateParams();
}

RLSWrenchObserver::~RLSWrenchObserver()
{
	perf_free(_cycle_perf);
}

bool RLSWrenchObserver::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("sensor_accel callback registration failed");
		return false;
	}

	return true;
}

void RLSWrenchObserver::updateParams()
{
	const float initial_guess[4] = {
				_param_rls_kf_init.get(),
				_param_rls_kr_init.get(),
				_param_rls_kf_init.get(),
				_param_rls_kr_init.get()
			};

	const float initial_confidence[4] = {
				_param_rls_kf_conf.get(),
				_param_rls_kr_conf.get(),
				_param_rls_kf_conf.get(),
				_param_rls_kr_conf.get()
			};

	const float R_diag[5] = {
				_param_rls_xy_noise.get(),
				_param_rls_xy_noise.get(),
				_param_rls_z_noise.get(),
				_param_rls_xy_noise.get(),
				_param_rls_xy_noise.get()
			};

	const struct VehicleParameters vehicle_params = {_param_rls_mass.get(), _param_rls_tilt.get(), _param_rls_n_rotors.get(), _param_rls_lpf_motor.get()};

	ModuleParams::updateParams();

	_identification.initialize(initial_guess, initial_confidence, R_diag, vehicle_params);
	_wrench_observer.initialize(_param_rls_lpf_force.get());

	PX4_INFO("UPDATED:\t%8.4f", (double)initial_guess[0]);
}

void RLSWrenchObserver::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;

			if (_landed) {
				_in_air = false;
			}
		}
	}

	if (!_sensor_accel_sub.updated()) {
		return;
	}

	sensor_accel_s accel{};
	_sensor_accel_sub.copy(&accel);

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s local_pos{};

		if (_vehicle_local_position_sub.copy(&local_pos)) {
			if (!_landed) {
				if (local_pos.dist_bottom > 0.3f) {
					_in_air = true;
				}
			}
		}
	}

	//actuator outputs required for each step
	if (!_actuator_outputs_sub.updated()) {
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	perf_begin(_cycle_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const float dt = (accel.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = accel.timestamp;

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	if (_armed && _in_air && (dt > 0.0002f) && (dt < 0.02f) && PX4_ISFINITE(accel.z)) {

		actuator_outputs_s actuator_outputs;
		vehicle_attitude_s v_att;

		if (_actuator_outputs_sub.copy(&actuator_outputs)) {

			const matrix::Vector3f y = matrix::Vector3f(accel.x, accel.y, accel.z);

			float speed[8] = {
				actuator_outputs.output[0] *_param_rls_speed_const.get(),
				actuator_outputs.output[1] *_param_rls_speed_const.get(),
				actuator_outputs.output[2] *_param_rls_speed_const.get(),
				actuator_outputs.output[3] *_param_rls_speed_const.get(),
				actuator_outputs.output[4] *_param_rls_speed_const.get(),
				actuator_outputs.output[5] *_param_rls_speed_const.get(),
				actuator_outputs.output[6] *_param_rls_speed_const.get(),
				actuator_outputs.output[7] *_param_rls_speed_const.get()
			};
			const bool flag = (hrt_absolute_time() > 40_s);
			const matrix::Vector<float, 8> output =  matrix::Vector<float, 8>(speed);
			matrix::Vector2f params_thrust = _identification.updateThrust(y, output, dt, flag);

			// const Vector<float, 8> y_lpf = _identification.getFilteredOutputs();
			const matrix::Vector3f p_error = _identification.getPredictionError();

			_vehicle_attitude_sub.update(&v_att);
			const matrix::Quatf q{v_att.q};

			//Compute external force and quaternion rotation from the FRD body frame to the NED earth frame
			matrix::Vector3f fe = q.conjugate(_wrench_observer.update(p_error, dt, flag));

			matrix::Vector2f params_offset = _identification.updateOffset(flag);

			//TODO: Check if rotation matrix necessary for lateral forces.
			//Continue implementing observer and switch for rls and force.
			PX4_INFO("Params :\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
					(double)params_thrust(0),
					(double)fe(2),
					(double)p_error(2),
					(double)flag,
					(double)((accel.z)*_param_rls_mass.get()),
					(double)dt,
					(double)params_offset(0));

			// Check validity of results
			bool valid = ((params_thrust(0) > 0.f) && (params_thrust(1) < params_thrust(0)));

			_valid_hysteresis.set_state_and_update(valid, actuator_outputs.timestamp);
			_valid = _valid_hysteresis.get_state();

			publishStatus(actuator_outputs.timestamp);

			//CHANGED DEBUG RATE TO 50HZ - CHANGE BACK
			//mavlink._main.cpp Line:1557
			debug_vect_s status_msg{};
			status_msg.x = fe(2);
			status_msg.y = p_error(2);
			status_msg.timestamp = hrt_absolute_time();
			_debug_vect_pub.publish(status_msg);
		}

	} else {
		_valid_hysteresis.set_state_and_update(false, hrt_absolute_time());

		if (_valid) {
			// only publish a single message to invalidate
			publishInvalidStatus();
			updateParams(); //reset RLS when landing
			_valid = false;
		}
	}

	perf_end(_cycle_perf);
}

void RLSWrenchObserver::publishStatus(const hrt_abstime &timestamp_sample)
{
	// hover_thrust_estimate_s status_msg{};

	// status_msg.timestamp_sample = timestamp_sample;

	// status_msg.hover_thrust = _hover_thrust_ekf.getHoverThrustEstimate();
	// status_msg.hover_thrust_var = _hover_thrust_ekf.getHoverThrustEstimateVar();

	// status_msg.accel_innov = _hover_thrust_ekf.getInnovation();
	// status_msg.accel_innov_var = _hover_thrust_ekf.getInnovationVar();
	// status_msg.accel_innov_test_ratio = _hover_thrust_ekf.getInnovationTestRatio();
	// status_msg.accel_noise_var = _hover_thrust_ekf.getAccelNoiseVar();

	// status_msg.valid = _valid;

	// status_msg.timestamp = hrt_absolute_time();

	// _hover_thrust_ekf_pub.publish(status_msg);

	// debug_vect_s status_msg{};
	// status_msg.x = 1.f;
	// status_msg.timestamp = hrt_absolute_time();
	// _debug_vect_pub.publish(status_msg);
}

void RLSWrenchObserver::publishInvalidStatus()
{
	// hover_thrust_estimate_s status_msg{};

	// status_msg.hover_thrust = NAN;
	// status_msg.hover_thrust_var = NAN;
	// status_msg.accel_innov = NAN;
	// status_msg.accel_innov_var = NAN;
	// status_msg.accel_innov_test_ratio = NAN;
	// status_msg.accel_noise_var = NAN;

	// _hover_thrust_ekf_pub.publish(status_msg);

	debug_vect_s status_msg{};
	status_msg.x = 0.f;
	status_msg.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(status_msg);
}

int RLSWrenchObserver::task_spawn(int argc, char *argv[])
{
	RLSWrenchObserver *instance = new RLSWrenchObserver();

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

int RLSWrenchObserver::print_status()
{
	perf_print_counter(_cycle_perf);
	return 0;
}

int RLSWrenchObserver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLSWrenchObserver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLS parameter identification and external wrench observer.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rls_wrench_observer", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rls_wrench_observer_main(int argc, char *argv[])
{
	return RLSWrenchObserver::main(argc, argv);
}
