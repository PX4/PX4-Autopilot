/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "SensorAirspeedSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

using namespace matrix;

SensorAirspeedSim::SensorAirspeedSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SensorAirspeedSim::~SensorAirspeedSim()
{
	perf_free(_loop_perf);
}

bool SensorAirspeedSim::init()
{
	ScheduleOnInterval(125_ms); // 8 Hz
	return true;
}

float SensorAirspeedSim::generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void SensorAirspeedSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
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
	}

	if (_sim_failure.get() == 0) {
		if (_vehicle_local_position_sub.updated() && _vehicle_global_position_sub.updated()
		    && _vehicle_attitude_sub.updated()) {

			vehicle_local_position_s lpos{};
			_vehicle_local_position_sub.copy(&lpos);

			vehicle_global_position_s gpos{};
			_vehicle_global_position_sub.copy(&gpos);

			vehicle_attitude_s attitude{};
			_vehicle_attitude_sub.copy(&attitude);

			Vector3f local_velocity = Vector3f{lpos.vx, lpos.vy, lpos.vz};
			Vector3f body_velocity = Dcmf{Quatf{attitude.q}} .transpose() * local_velocity;

			// device id
			device::Device::DeviceId device_id;
			device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
			device_id.devid_s.bus = 0;
			device_id.devid_s.address = 0;
			device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;

			const float alt_amsl = gpos.alt;
			const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;
			const float density_ratio = powf(TEMPERATURE_MSL / temperature_local, 4.256f);
			const float air_density = AIR_DENSITY_MSL / density_ratio;

			// calculate differential pressure + noise in hPa
			const float diff_pressure_noise = (float)generate_wgn() * 0.01f;
			float diff_pressure = sign(body_velocity(0)) * 0.005f * air_density  * body_velocity(0) * body_velocity(
						      0) + diff_pressure_noise;


			differential_pressure_s differential_pressure{};
			// report.timestamp_sample = time;
			differential_pressure.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
			differential_pressure.differential_pressure_pa = (double)diff_pressure * 100.0; // hPa to Pa;
			differential_pressure.temperature = temperature_local;
			differential_pressure.timestamp = hrt_absolute_time();
			_differential_pressure_pub.publish(differential_pressure);

		}
	}

	perf_end(_loop_perf);
}

int SensorAirspeedSim::task_spawn(int argc, char *argv[])
{
	SensorAirspeedSim *instance = new SensorAirspeedSim();

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

int SensorAirspeedSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAirspeedSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_arispeed_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_airspeed_sim_main(int argc, char *argv[])
{
	return SensorAirspeedSim::main(argc, argv);
}
