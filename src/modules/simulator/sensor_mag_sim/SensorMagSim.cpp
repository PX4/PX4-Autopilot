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

#include "SensorMagSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>

using namespace matrix;

SensorMagSim::SensorMagSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_MAGSIM);
	_px4_mag.set_external(false);
}

SensorMagSim::~SensorMagSim()
{
	perf_free(_loop_perf);
}

bool SensorMagSim::init()
{
	ScheduleOnInterval(20_ms); // 50 Hz
	return true;
}

float SensorMagSim::generate_wgn()
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
			float U1 = (float)rand() / RAND_MAX;
			float U2 = (float)rand() / RAND_MAX;
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

void SensorMagSim::Run()
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

	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s gpos;

		if (_vehicle_global_position_sub.copy(&gpos)) {
			if (gpos.eph < 1000) {

				// magnetic field data returned by the geo library using the current GPS position
				const float mag_declination_gps = get_mag_declination_radians(gpos.lat, gpos.lon);
				const float mag_inclination_gps = get_mag_inclination_radians(gpos.lat, gpos.lon);
				const float mag_strength_gps = get_mag_strength_gauss(gpos.lat, gpos.lon);

				_mag_earth_pred = Dcmf(Eulerf(0, -mag_inclination_gps, mag_declination_gps)) * Vector3f(mag_strength_gps, 0, 0);

				_mag_earth_available = true;
			}
		}
	}

	if (_mag_earth_available) {
		vehicle_attitude_s attitude;

		if (_vehicle_attitude_sub.update(&attitude)) {
			Vector3f expected_field = Dcmf{Quatf{attitude.q}} .transpose() * _mag_earth_pred;

			expected_field += noiseGauss3f(0.02f, 0.02f, 0.03f);

			_px4_mag.update(attitude.timestamp,
					expected_field(0) + _sim_mag_offset_x.get(),
					expected_field(1) + _sim_mag_offset_y.get(),
					expected_field(2) + _sim_mag_offset_z.get());
		}
	}
}

int SensorMagSim::task_spawn(int argc, char *argv[])
{
	SensorMagSim *instance = new SensorMagSim();

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

int SensorMagSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorMagSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_mag_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_mag_sim_main(int argc, char *argv[])
{
	return SensorMagSim::main(argc, argv);
}
