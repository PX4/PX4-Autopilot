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

#include <lib/atmosphere/atmosphere.h>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingWindEstimator::FixedwingWindEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
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
	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		// get rotation between NED frames
		_attitude = Quatf(vehicle_attitude.q);
	}
}

void
FixedwingWindEstimator::vehicle_local_position_poll()
{
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		// get rotation between NED frames
		_local_velocity = Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
	}
}

void
FixedwingWindEstimator::vehicle_acceleration_poll()
{
	vehicle_acceleration_s vehicle_acceleration;

	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {
		_acceleration = Vector3f(vehicle_acceleration.xyz);
	}
}

matrix::Vector3f FixedwingWindEstimator::predictBodyAirVelocity()
{
	// Get Aerodynamic coefficients
	const float wing_area = _param_fw_w_area.get();
	const float C_B1 = _param_fw_w_c_b1.get();
	const float C_A0 = _param_fw_w_c_a0.get();
	const float C_A1 = _param_fw_w_c_a1.get();
	const float mass = _param_fw_w_mass.get();
	const float stall_airspeed = _param_fw_airspd_stall.get();

	// compute expected AoA from g-forces:
	Vector3f body_force = mass * (_acceleration + _attitude.rotateVectorInverse(Vector3f(0.f, 0.f, CONSTANTS_ONE_G)));

	const float speed = fmaxf(_calibrated_airspeed, stall_airspeed);
	const float dynamic_force = 0.5f * atmosphere::kAirDensitySeaLevelStandardAtmos * powf(speed, 2) * wing_area;
	float u_approx = _true_airspeed;
	float v_approx = -body_force(1) * _true_airspeed / (dynamic_force * C_B1);
	float w_approx = (body_force(2) * _true_airspeed / dynamic_force  + C_A0) / C_A1;
	Vector3f vel_air(u_approx, v_approx, w_approx);
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
		vehicle_local_position_poll();

		const float stall_airspeed = _param_fw_airspd_stall.get();

		// Do not compute wind estimate under stall speed
		if (_calibrated_airspeed > stall_airspeed) {
			matrix::Vector3f air_velocity_body = predictBodyAirVelocity();

			matrix::Vector3f air_velocity_local = _attitude.rotateVector(air_velocity_body);

			// compute wind from wind triangle
			matrix::Vector3f wind = _local_velocity - air_velocity_local;
			airflow_s airflow_msg;
			airflow_msg.timestamp = hrt_absolute_time();
			airflow_msg.u = air_velocity_body(0);
			airflow_msg.v = air_velocity_body(1);
			airflow_msg.w = air_velocity_body(2);
			airflow_msg.windspeed_north = wind(0);
			airflow_msg.windspeed_east = wind(1);
			airflow_msg.windspeed_down = wind(2);
			_airflow_pub.publish(airflow_msg);
		}

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
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
	FixedwingWindEstimator *instance = new FixedwingWindEstimator();

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
fw_wind_estimator is a 3D wind estimator for fixed-wing vehicles.

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
