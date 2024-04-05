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

#include "SensorAirflowSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

using namespace matrix;

SensorAirflowSim::SensorAirflowSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SensorAirflowSim::~SensorAirflowSim()
{
	perf_free(_loop_perf);
}

bool SensorAirflowSim::init()
{
	_sensor_airflow_pub.advertise();
	ScheduleOnInterval(100_ms); // 10 Hz
	return true;
}

float SensorAirflowSim::generate_wgn()
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

void SensorAirflowSim::Run()
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

		_offsets = Vector3f(_x_offset_param.get(), 0.f, _z_offset_param.get());
	}

	/* we should check whether the wind_topic has been updated, but currently gz
	 * does not publish the wind topics periodically. We need to publish to the wind topic
	 * using the following command:
	 * gz topic -t "/world/$world/wind/" -m gz.msgs.Wind  -p "linear_velocity: {x:0, y:5}, enable_wind: false"
	*/
	if (_vehicle_local_position_sub.updated() && _vehicle_local_position_sub.updated()
	    && _vehicle_attitude_sub.updated()) {

		vehicle_local_position_s lpos{};
		_vehicle_local_position_sub.copy(&lpos);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		vehicle_attitude_s attitude{};
		_vehicle_attitude_sub.copy(&attitude);

		wind_s wind{};
		_wind_sub.copy(&wind);

		Vector3f local_velocity = Vector3f{lpos.vx, lpos.vy, lpos.vz};
		Vector3f body_velocity = Dcmf{Quatf{attitude.q}} .transpose() * local_velocity;

		Vector3f body_angular_vel = Vector3f(angular_velocity.xyz);
		Vector3f wind_velocity = Vector3f(wind.windspeed_north, wind.windspeed_east, 0);
		Vector3f body_wind_velocity = Dcmf{Quatf{attitude.q}} .transpose() * wind_velocity;

		// device id
		device::Device::DeviceId device_id;
		device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
		device_id.devid_s.bus = 0;
		device_id.devid_s.address = 0;
		device_id.devid_s.devtype = DRV_AIRFLOW_DEVTYPE_SIM;

		// calculate measured airflow due to body rotational velocity because the sensor sits at an
		// offset from the CoG
		Vector3f offset_velocity = body_angular_vel.cross(_offsets);
		Vector3f measured_airflow = body_velocity + offset_velocity + body_wind_velocity;

		// add noise to sensor measurement
		Vector3f measurement_noise = noiseGauss3f(0.3, 4 * M_PI_F / 180.f, 0);

		// For now the sensor cant measure airflow in the Z direction.
		measured_airflow(2) = 0.0f;
		float airflow_speed = measured_airflow.norm() + measurement_noise(0);
		float airflow_direction  = atan2f(measured_airflow(1), measured_airflow(0)) + measurement_noise(1);

		sensor_airflow_s sensor_airflow{};
		sensor_airflow.timestamp = hrt_absolute_time();
		sensor_airflow.device_id = 0;
		sensor_airflow.speed = airflow_speed;
		sensor_airflow.direction = airflow_direction;
		sensor_airflow.status = 0; // status of zero means no error

		_sensor_airflow_pub.publish(sensor_airflow);

	}

	perf_end(_loop_perf);
}

int SensorAirflowSim::task_spawn(int argc, char *argv[])
{
	SensorAirflowSim *instance = new SensorAirflowSim();

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

int SensorAirflowSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAirflowSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_airflow_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_airflow_sim_main(int argc, char *argv[])
{
	return SensorAirflowSim::main(argc, argv);
}
