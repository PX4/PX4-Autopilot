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
 * @file RLSWrenchEstimator.cpp
 * @brief RLS parameter identification and external wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#include "RLSWrenchEstimator.hpp"

RLSWrenchEstimator::RLSWrenchEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_valid_hysteresis.set_hysteresis_time_from(false, 2_s);
	updateParams();
}

RLSWrenchEstimator::~RLSWrenchEstimator()
{
	perf_free(_cycle_perf);
}

bool RLSWrenchEstimator::init()
{
	// execute Run() on every vehicle_acceleration publication
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("vehicle_acceleration callback registration failed");
		return false;
	}

	return true;
}

void RLSWrenchEstimator::updateParams()
{
	ModuleParams::updateParams();

	if (!_in_air) {
		const float initial_guess[4] = {
			_param_rls_kf_init.get(),
			_param_rls_kr_init.get(),
			_param_rls_xo_init.get() * 1000,
			_param_rls_yo_init.get() * 1000
		};

		const float initial_confidence[4] = {
			_param_rls_kf_conf.get(),
			_param_rls_kr_conf.get(),
			_param_rls_xo_conf.get(),
			_param_rls_yo_conf.get()
		};

		const float R_diag[5] = {
			_param_rls_xy_noise.get(),
			_param_rls_xy_noise.get(),
			_param_rls_z_noise.get(),
			_param_rls_f_noise.get(),
			_param_rls_f_noise.get()
		};

		const struct VehicleParameters vehicle_params = {_param_rls_mass.get(),
								math::radians(_param_rls_tilt.get()),
								_param_rls_n_rotors.get(),
								_param_rls_lpf_motor.get(),
								_param_rls_km.get(),
								_param_rls_diameter.get(),
								_param_rls_top_height.get(),
								_param_rls_bot_height.get()
		};

		const Vector3f inertia_diag = {
			_param_rls_inertia_x.get()*(1E3f),
			_param_rls_inertia_y.get()*(1E3f),
			_param_rls_inertia_z.get()*(1E3f)
		};

		_identification.initialize(initial_guess, initial_confidence, R_diag, vehicle_params);
		_wrench_estimator.initialize(_param_rls_lpf_force.get(), _param_rls_lpf_moment.get(), inertia_diag);

	}

}

void RLSWrenchEstimator::Run()
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

	if (!_vehicle_acceleration_sub.updated()) {
		return;
	}

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

	// //all inputs required for each step
	// if (!_actuator_outputs_sub.updated() || !_vehicle_attitude_sub.updated() || !_vehicle_angular_velocity_sub.updated()) {
	// 	return;
	// }

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	perf_begin(_cycle_perf);

	vehicle_acceleration_s accel;
	actuator_outputs_s actuator_outputs;
	vehicle_attitude_s v_att;
	vehicle_angular_velocity_s v_ang_vel;
	battery_status_s batt_stat;

	_finite = copyAndCheckAllFinite(accel, actuator_outputs, v_att, v_ang_vel, batt_stat);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const float dt = (accel.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = accel.timestamp;

	if (_debug_vect_sub.updated()) {
		debug_vect_s flags_vect;
		_debug_vect_sub.copy(&flags_vect);
		_interaction_flag = (flags_vect.x > 0.5f);

		_debug_timestamp_last = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_debug_timestamp_last) > 1_s) {
		_interaction_flag = false; //timeout case external link lost
	}

// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	if (_finite && _armed && _in_air && (dt > 0.0002f) && (dt < 0.02f)) {

		const matrix::Vector3f acc = matrix::Vector3f(accel.xyz[0], accel.xyz[1], accel.xyz[2]);

		_voltage = math::constrain(batt_stat.voltage_filtered_v, _param_n_cells.get() * 3.0f, _param_n_cells.get() * 4.2f);

		float speed[8] = {
			((actuator_outputs.output[0] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[1] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[2] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[3] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[4] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[5] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[6] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
			((actuator_outputs.output[7] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get()) * sqrtf(powf((_voltage / _param_rls_speed_v1.get()),_param_rls_speed_v2.get())),
		};

		const matrix::Vector<float, 8> output =  matrix::Vector<float, 8>(speed);

		//RLS Thrust
		_identification.updateThrust(acc, output, dt, _interaction_flag);

		const matrix::Vector3f p_error_t = _identification.getPredictionErrorThrust();
		const matrix::Quatf q{v_att.q};

		//Wrench Estimator Thrust
		_wrench_estimator.updateForce(p_error_t, dt, _interaction_flag);
		// --------------------------------------------------------- //

		//RLS Offset
		_identification.updateOffset(q, _interaction_flag);

		matrix::Vector3f p_error_o = _identification.getPredictionErrorOffset();

		//Wrench Estimator Moment
		_wrench_estimator.updateMoment(p_error_o, matrix::Vector3f(v_ang_vel.xyz), dt, _interaction_flag);

		// Check validity of results - IMPROVE
		// bool valid = ((params_thrust(0) > 0.f) && (params_thrust(1) < params_thrust(0)));
		bool valid = true;

		_valid_hysteresis.set_state_and_update(valid, accel.timestamp);
		_valid = _valid_hysteresis.get_state();

		publishStatus();

	} else {
		_valid_hysteresis.set_state_and_update(false, hrt_absolute_time());

		publishInvalidStatus();
		if (_valid) {
			updateParams(); //reset RLS when landing
			_valid = false;
		}
	}

	perf_end(_cycle_perf);
}

void RLSWrenchEstimator::publishStatus()
{
	rls_wrench_estimator_s status_msg{};
	matrix::Vector3f Fe = _wrench_estimator.getExternalForce();
	matrix::Vector3f Me = _wrench_estimator.getExternalMoment();
	matrix::Vector2f params_thrust = _identification.getEstimationThrust();
	matrix::Vector3f params_offset = _identification.getEstimationOffset();
	Vector3f Fi = _identification.getActuatorForceVector();
	Vector3f Mi = _identification.getActuatorMomentVector();

	status_msg.timestamp = hrt_absolute_time();

	status_msg.fe[0] = Fe(0);
	status_msg.fe[1] = Fe(1);
	status_msg.fe[2] = Fe(2);

	status_msg.me[0] = Me(0);
	status_msg.me[1] = Me(1);
	status_msg.me[2] = Me(2);

	status_msg.x_thrust[0] = params_thrust(0);
	status_msg.x_thrust[1] = params_thrust(1);

	status_msg.x_offset[0] = params_offset(0);
	status_msg.x_offset[1] = params_offset(1);
	status_msg.x_offset[2] = params_offset(2);

	status_msg.fi[0] = Fi(0);
	status_msg.fi[1] = Fi(1);
	status_msg.fi[2] = Fi(2);

	status_msg.mi[0] = Mi(0);
	status_msg.mi[1] = Mi(1);
	status_msg.mi[2] = Mi(2);

	status_msg.interaction_flag = _interaction_flag;
	status_msg.valid = _valid;

	_rls_wrench_estimator_pub.publish(status_msg);
}

void RLSWrenchEstimator::publishInvalidStatus()
{
	rls_wrench_estimator_s status_msg{};

	status_msg.timestamp = hrt_absolute_time();

	status_msg.fe[0] = 0.f;
	status_msg.fe[1] = 0.f;
	status_msg.fe[2] = 0.f;

	status_msg.me[0] = 0.f;
	status_msg.me[1] = 0.f;
	status_msg.me[2] = 0.f;

	status_msg.x_thrust[0] = NAN;
	status_msg.x_thrust[1] = NAN;

	status_msg.x_offset[0] = NAN;
	status_msg.x_offset[1] = NAN;
	status_msg.x_offset[2] = NAN;

	status_msg.fi[0] = NAN;
	status_msg.fi[1] = NAN;
	status_msg.fi[2] = NAN;

	status_msg.mi[0] = NAN;
	status_msg.mi[1] = NAN;
	status_msg.mi[2] = NAN;

	status_msg.interaction_flag = _interaction_flag;
	status_msg.valid = false;

	_rls_wrench_estimator_pub.publish(status_msg);
}

bool RLSWrenchEstimator::copyAndCheckAllFinite(vehicle_acceleration_s &accel, actuator_outputs_s &actuator_outputs,
		vehicle_attitude_s &v_att, vehicle_angular_velocity_s &v_ang_vel, battery_status_s &batt_stat)
{
	_vehicle_acceleration_sub.copy(&accel);

	if (!(PX4_ISFINITE(accel.xyz[0]) && PX4_ISFINITE(accel.xyz[1]) && PX4_ISFINITE(accel.xyz[2]))) {

		return false;
	}


	_vehicle_attitude_sub.copy(&v_att);

	if (!(PX4_ISFINITE(v_att.q[0]) && PX4_ISFINITE(v_att.q[1]) && PX4_ISFINITE(v_att.q[2]) && PX4_ISFINITE(v_att.q[3]))) {

		return false;
	}

	_vehicle_angular_velocity_sub.copy(&v_ang_vel);

	if (!(PX4_ISFINITE(v_ang_vel.xyz[0]) && PX4_ISFINITE(v_ang_vel.xyz[1]) && PX4_ISFINITE(v_ang_vel.xyz[2]))) {

		return false;
	}

	_battery_status_sub.copy(&batt_stat);

	if (!(PX4_ISFINITE(batt_stat.voltage_v))) {

		_voltage = 11.7f;
	}


	_actuator_outputs_sub.copy(&actuator_outputs);

	for (int i = 0; i < 8; i++) {
		if (!PX4_ISFINITE(actuator_outputs.output[i])) {return false;}
	}

	return true;
}

int RLSWrenchEstimator::task_spawn(int argc, char *argv[])
{
	RLSWrenchEstimator *instance = new RLSWrenchEstimator();

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

int RLSWrenchEstimator::print_status()
{
	perf_print_counter(_cycle_perf);
	return 0;
}

int RLSWrenchEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLSWrenchEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLS parameter identification and external wrench estimator.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rls_wrench_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rls_wrench_estimator_main(int argc, char *argv[])
{
	return RLSWrenchEstimator::main(argc, argv);
}
