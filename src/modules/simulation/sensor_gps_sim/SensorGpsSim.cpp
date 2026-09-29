/****************************************************************************
 *
 *   Copyright (c) 2021-2026 PX4 Development Team. All rights reserved.
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

#include "SensorGpsSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

using namespace matrix;

ModuleBase::Descriptor SensorGpsSim::desc{task_spawn, custom_command, print_usage};

SensorGpsSim::SensorGpsSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SensorGpsSim::~SensorGpsSim()
{
	perf_free(_loop_perf);
}

bool SensorGpsSim::init()
{
	ScheduleOnInterval(125_ms); // 8 Hz
	return true;
}

float SensorGpsSim::generate_wgn()
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

void SensorGpsSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
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

	updateFailureConfig();
	const bool rtk = updateRtcmCorrections();

	if (_vehicle_local_position_sub.updated() && _vehicle_global_position_sub.updated()) {

		vehicle_local_position_s lpos{};
		_vehicle_local_position_sub.copy(&lpos);

		vehicle_global_position_s gpos{};
		_vehicle_global_position_sub.copy(&gpos);

		// Correlated Markov process position noise (matching GZBridge model)
		_gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n +
				   _pos_random_walk * generate_wgn() * _pos_noise_amplitude;

		_gps_pos_noise_e = _pos_markov_time * _gps_pos_noise_e +
				   _pos_random_walk * generate_wgn() * _pos_noise_amplitude;

		_gps_pos_noise_d = _pos_markov_time * _gps_pos_noise_d +
				   _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f;

		const double latitude = gpos.lat + math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);
		const double longitude = gpos.lon + math::degrees((double)_gps_pos_noise_e / CONSTANTS_RADIUS_OF_EARTH);
		const double altitude = (double)(gpos.alt + _gps_pos_noise_d);

		_gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n +
				   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

		_gps_vel_noise_e = _vel_markov_time * _gps_vel_noise_e +
				   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

		_gps_vel_noise_d = _vel_markov_time * _gps_vel_noise_d +
				   _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

		const Vector3f gps_vel = Vector3f{lpos.vx + _gps_vel_noise_n, lpos.vy + _gps_vel_noise_e, lpos.vz + _gps_vel_noise_d};

		// device id
		device::Device::DeviceId device_id;
		device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
		device_id.devid_s.bus = 0;
		device_id.devid_s.address = 0;
		device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

		sensor_gnss_s sensor_gnss{};

		if (_sim_gps_used.get() >= 4) {
			// fix: RTK fixed while corrections are flowing, 3D otherwise
			sensor_gnss.fix_type = rtk ? sensor_gnss_s::FIX_TYPE_RTK_FIXED : sensor_gnss_s::FIX_TYPE_3D;
			sensor_gnss.speed_accuracy = 0.4f;
			sensor_gnss.course_accuracy = 0.1f;
			sensor_gnss.eph = rtk ? 0.02f : 0.9f;
			sensor_gnss.epv = rtk ? 0.04f : 1.78f;
			sensor_gnss.hdop = 0.7f;
			sensor_gnss.vdop = 1.1f;

		} else {
			// no fix
			sensor_gnss.fix_type = 0; // No fix
			sensor_gnss.speed_accuracy = 100.f;
			sensor_gnss.course_accuracy = 100.f;
			sensor_gnss.eph = 100.f;
			sensor_gnss.epv = 100.f;
			sensor_gnss.hdop = 100.f;
			sensor_gnss.vdop = 100.f;
		}

		sensor_gnss.timestamp_sample = gpos.timestamp_sample;
		sensor_gnss.time_utc_usec = 0;
		sensor_gnss.device_id = device_id.devid;
		sensor_gnss.latitude = latitude; // Latitude in degrees
		sensor_gnss.longitude = longitude; // Longitude in degrees
		sensor_gnss.altitude_msl = altitude; // Altitude in meters above MSL
		sensor_gnss.altitude_ellipsoid = altitude;
		sensor_gnss.noise = 0;
		sensor_gnss.jamming_indicator = 0;
		sensor_gnss.ground_speed = sqrtf(gps_vel(0) * gps_vel(0) + gps_vel(1) * gps_vel(1)); // GPS ground speed, (metres/sec)
		sensor_gnss.vel_north = gps_vel(0);
		sensor_gnss.vel_east = gps_vel(1);
		sensor_gnss.vel_down = gps_vel(2);
		sensor_gnss.course = atan2(gps_vel(1),
					   gps_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
		sensor_gnss.timestamp_time_relative = 0;
		sensor_gnss.automatic_gain_control = 0;
		sensor_gnss.jamming_state = 0;
		sensor_gnss.spoofing_state = 0;
		sensor_gnss.vel_ned_valid = true;
		sensor_gnss.satellites_used = _sim_gps_used.get();

		publishWithFailures(0, sensor_gnss, _sensor_gnss_pub);

		const float gnss1_offx = _param_gnss1_offx.get();
		const float gnss1_offy = _param_gnss1_offy.get();

		if (fabsf(gnss1_offx) > 0.f || fabsf(gnss1_offy) > 0.f) {
			sensor_gnss_s gnss1 = sensor_gnss;

			device_id.devid_s.address = 1;
			gnss1.device_id = device_id.devid;

			gnss1.latitude  = latitude  + (double)gnss1_offx / CONSTANTS_RADIUS_OF_EARTH * (180.0 / M_PI);
			gnss1.longitude = longitude + (double)gnss1_offy / CONSTANTS_RADIUS_OF_EARTH * (180.0 / M_PI) / cos(latitude * M_PI / 180.0);

			publishWithFailures(1, gnss1, _sensor_gnss_pub2);
		}
	}

	perf_end(_loop_perf);
}

void SensorGpsSim::publishWithFailures(int instance, sensor_gnss_s gnss, uORB::PublicationMulti<sensor_gnss_s> &pub)
{
	gnss.timestamp = hrt_absolute_time();

	if (!failure_injection::process_gnss(_failure_config, instance, gnss, _stuck[instance])) {
		return;
	}

	pub.publish(gnss);
}

void SensorGpsSim::updateFailureConfig()
{
	_failure_config.update();
}

bool SensorGpsSim::updateRtcmCorrections()
{
	rtcm_data_s msg;

	for (int instance = 0; instance < _rtcm_corrections_sub.size(); instance++) {
		while (_rtcm_corrections_sub[instance].update(&msg)) {
			_last_rtcm_time = math::max(_last_rtcm_time, msg.timestamp);
		}
	}

	return (_last_rtcm_time != 0) && (hrt_elapsed_time(&_last_rtcm_time) < RTCM_TIMEOUT);
}

int SensorGpsSim::task_spawn(int argc, char *argv[])
{
	SensorGpsSim *instance = new SensorGpsSim();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SensorGpsSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorGpsSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_gps_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_gps_sim_main(int argc, char *argv[])
{
	return ModuleBase::main(SensorGpsSim::desc, argc, argv);
}
