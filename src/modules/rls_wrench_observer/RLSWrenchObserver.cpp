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
	updateParams();
}

RLSWrenchObserver::~RLSWrenchObserver()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
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
	const float rls_pmen_noise = _param_rls_pmen_noise.get();
	ModuleParams::updateParams();

	const matrix::Vector2f X_O = matrix::Vector2f (1.f,0.f);
	const float a = 1.0f;
	const float b = 0.1f;

	_identification.initialize(X_O,a,b);

	const matrix::Vector3f Yvec = matrix::Vector3f(1.f,2.f,3.f);
	matrix::Matrix<float, 3, 2> Hmatrix;
	matrix::Vector2f par;

	Hmatrix.setAll(1);

	par = _identification.update(Yvec,Hmatrix);

	PX4_INFO("Updated %8.4f",(double)(par(1)));
	PX4_INFO("Updated %8.4f",(double)(rls_pmen_noise+_identification.getThrustConstant()));
}

void RLSWrenchObserver::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	// Example
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		const matrix::Vector3f Yvec = matrix::Vector3f(1.f,2.f,3.f);
		matrix::Matrix<float, 3, 2> Hmatrix;
		matrix::Vector2f par;

		if (_sensor_accel_sub.copy(&accel)) {


			Hmatrix.setAll(accel.z);

			par = _identification.update(Yvec,Hmatrix);

		// PX4_INFO("Sensor_accel:\t%8.4f\t%8.4f",
		// 		(double)par(0),
		// 		(double)par(1));

		// PX4_INFO("Sensor_accel:\t%8.4f\t%8.4f\t%8.4f",
		// 		(double)accel.x,
		// 		(double)accel.y,
		// 		(double)accel.z);


		}
	}

if (_actuator_outputs_sub.updated()) {
		actuator_outputs_s actuator_outputs;

		if (_actuator_outputs_sub.copy(&actuator_outputs)) {

		// PX4_INFO("actuator_output:\t%8.4f",
		// 		(double)actuator_outputs.output[0]);
		}
	}

	// if (!_vehicle_local_position_sub.updated()) {
	// 	return;
	// }

	// vehicle_local_position_s local_pos{};


		// if (_vehicle_local_position_sub.copy(&local_pos)) {
		// 	PX4_INFO("vehicle_local_position_accel:\t%8.4f\t%8.4f\t%8.4f",
		// 			(double)local_pos.ax,
		// 			(double)local_pos.ay,
		// 			(double)local_pos.az);
		// }

	// if (!_sensor_combined_sub.updated()) {
	// 	return;
	// }
	// sensor_combined_s sensor_combined;


	// 	if (_sensor_combined_sub.copy(&sensor_combined)) {
	// 		PX4_INFO("combined_accel:\t%8.4f\t%8.4f\t%8.4f",		
	// 				(double)sensor_combined.accelerometer_m_s2[0],
	// 				(double)sensor_combined.accelerometer_m_s2[1],
	// 				(double)sensor_combined.accelerometer_m_s2[2]);
	// 	}


	// Example
	//  publish some data
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);

	perf_end(_loop_perf);
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
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
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
Example of a simple module running out of a work queue PMEN UPxx.

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
