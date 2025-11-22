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

#include "FixedwingWindEstimator.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingWindEstimator::FixedwingWindEstimator(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_handle_param_vt_fw_difthr_en = param_find("VT_FW_DIFTHR_EN");

	/* fetch initial parameter values */
	parameters_update();

}

FixedwingWindEstimator::~FixedwingWindEstimator()
{
	perf_free(_loop_perf);
}

bool
FixedwingWindEstimator::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingWindEstimator::parameters_update()
{
	if (_handle_param_vt_fw_difthr_en != PARAM_INVALID) {
		param_get(_handle_param_vt_fw_difthr_en, &_param_vt_fw_difthr_en);
	}


	return PX4_OK;
}

void
FixedwingWindEstimator::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

void FixedwingWindEstimator::airspeed_poll()
{
	airspeed_validated_s airspeed_validated;

	if (_airspeed_validated_sub.update(&airspeed_validated)) {
		_calibrated_airspeed = math::max(0.5f, airspeed_validated.calibrated_airspeed_m_s);
		_true_airspeed = math::max(0.5f, airspeed_validated.true_airspeed_m_s);
	}
}


void
FixedwingWindEstimator::vehicle_attitude_poll()
{
	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		// get rotation between NED frames
		_attitude = Quatf(_vehicle_attitude.q);
	}
}

void
FixedwingWindEstimator::vehicle_acceleration_poll()
{
	//vehicle_local_position_s pos;
	///TODO: We should probably get it from the imu, not the local one?
	vehicle_acceleration_s vehicle_acceleration;
	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)){
		Dcmf R_ib(_attitude);
		_acceleration = R_ib*Vector3f(vehicle_acceleration.xyz) - _gravity;
	}
}

matrix::Vector3f FixedwingWindEstimator::compute_wind_estimate()
{
	float _stall_airspeed{1.0f};
	float _rho{1.225};
	float _area{1.0};
	float _C_B1{1.0};
	float _mass{1.0};

	Dcmf R_ib(_attitude);
	Dcmf R_bi(R_ib.transpose());
	// compute expected AoA from g-forces:
	matrix::Vector3f body_force = _mass * R_bi * (_acceleration + _gravity);

	// ***************** NEW COMPUTATION FROM MATLAB CALIBRATION **********************
	float speed = fmaxf(_calibrated_airspeed, _stall_airspeed);
	float u_approx = _true_airspeed;
	float v_approx = body_force(1) * _true_airspeed / (0.5f * _rho * powf(speed, 2) * _area * _C_B1);
	float w_approx = (-body_force(2) * _true_airspeed / (0.5f * _rho * powf(speed, 2) * _area) - 0.1949f) / 3.5928f;
	matrix::Vector3f vel_air = R_ib * (matrix::Vector3f{u_approx, v_approx, w_approx});

	return vel_air;
}

void FixedwingWindEstimator::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		vehicle_land_detected_poll();
		airspeed_poll();
		vehicle_attitude_poll();

		matrix::Vector3f air_velocity = compute_wind_estimate();

		// compute wind from wind triangle
		matrix::Vector3f _vehcile_local_velocity;
		matrix::Vector3f wind = _vehcile_local_velocity - air_velocity;
		PX4_INFO("wind estimate: \t%.1f, \t%.1f, \t%.1f", (double)wind(0), (double)wind(1), (double)wind(2));

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingWindEstimator::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingWindEstimator *instance = new FixedwingWindEstimator(vtol);

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

int FixedwingWindEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingWindEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_wind_estimator_main(int argc, char *argv[])
{
	return FixedwingWindEstimator::main(argc, argv);
}
