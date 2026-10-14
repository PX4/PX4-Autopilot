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

#include "SensorGnssSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ModuleBase::Descriptor SensorGnssSim::desc{task_spawn, custom_command, print_usage};

SensorGnssSim::SensorGnssSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SensorGnssSim::~SensorGnssSim()
{
	perf_free(_loop_perf);
}

bool SensorGnssSim::init()
{
	ScheduleOnInterval(125_ms); // 8 Hz
	return true;
}

float SensorGnssSim::generate_wgn()
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

void SensorGnssSim::Run()
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

	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.update(&attitude)) {
		_attitude = matrix::Quatf(attitude.q);
	}

	if (_vehicle_local_position_sub.updated() && _vehicle_global_position_sub.updated()) {

		vehicle_local_position_s lpos{};
		_vehicle_local_position_sub.copy(&lpos);

		vehicle_global_position_s gpos{};
		_vehicle_global_position_sub.copy(&gpos);

		const Dcmf body_to_ned{_attitude};
		const int receivers = math::constrain(static_cast<int>(_sim_gnss_num.get()), 1, GNSS_MAX_INSTANCES);

		const Vector3f biases[GNSS_MAX_INSTANCES] {
			{_param_sim_gnss0_bias_n.get(), _param_sim_gnss0_bias_e.get(), _param_sim_gnss0_bias_d.get()},
			{_param_sim_gnss1_bias_n.get(), _param_sim_gnss1_bias_e.get(), _param_sim_gnss1_bias_d.get()},
		};

		for (int instance = 0; instance < receivers; instance++) {
			ReceiverNoise &noise = _noise[instance];

			// Correlated Markov process position noise (matching GZBridge model)
			noise.position(0) = _pos_markov_time * noise.position(0) + _pos_random_walk * generate_wgn() * _pos_noise_amplitude;
			noise.position(1) = _pos_markov_time * noise.position(1) + _pos_random_walk * generate_wgn() * _pos_noise_amplitude;
			noise.position(2) = _pos_markov_time * noise.position(2)
					    + _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f;

			noise.velocity(0) = _vel_markov_time * noise.velocity(0) + _vel_noise_density * generate_wgn() * _vel_noise_amplitude;
			noise.velocity(1) = _vel_markov_time * noise.velocity(1) + _vel_noise_density * generate_wgn() * _vel_noise_amplitude;
			noise.velocity(2) = _vel_markov_time * noise.velocity(2)
					    + _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

			// The antenna sits at its lever arm from the centre of gravity, and the receiver reports it with its own error
			const Vector3f position_error = body_to_ned * antennaOffset(instance) + biases[instance] + noise.position;

			const double latitude = gpos.lat + math::degrees((double)position_error(0) / CONSTANTS_RADIUS_OF_EARTH);
			const double longitude = gpos.lon + math::degrees((double)position_error(1) / CONSTANTS_RADIUS_OF_EARTH)
						 / cos(math::radians(gpos.lat));
			const double altitude = (double)(gpos.alt - position_error(2));

			const Vector3f gnss_vel = Vector3f{lpos.vx, lpos.vy, lpos.vz} + noise.velocity;

			// device id
			device::Device::DeviceId device_id;
			device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
			device_id.devid_s.bus = 0;
			device_id.devid_s.address = instance;
			device_id.devid_s.devtype = DRV_GNSS_DEVTYPE_SIM;

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
			sensor_gnss.ground_speed = sqrtf(gnss_vel(0) * gnss_vel(0) + gnss_vel(1) * gnss_vel(1)); // GNSS ground speed, (metres/sec)
			sensor_gnss.vel_north = gnss_vel(0);
			sensor_gnss.vel_east = gnss_vel(1);
			sensor_gnss.vel_down = gnss_vel(2);
			sensor_gnss.course = atan2(gnss_vel(1),
						   gnss_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
			sensor_gnss.timestamp_time_relative = 0;
			sensor_gnss.automatic_gain_control = 0;
			sensor_gnss.jamming_state = 0;
			sensor_gnss.spoofing_state = 0;
			sensor_gnss.vel_ned_valid = true;
			sensor_gnss.satellites_used = _sim_gps_used.get();

			publishWithFailures(instance, sensor_gnss);
			publishRelativeHeading(instance, sensor_gnss, body_to_ned);
		}
	}

	perf_end(_loop_perf);
}

void SensorGnssSim::publishWithFailures(int instance, sensor_gnss_s gnss)
{
	uORB::PublicationMulti<sensor_gnss_s> &pub = _sensor_gnss_pub[instance];
	gnss.timestamp = hrt_absolute_time();

	if (!failure_injection::process_gnss(_failure_config, pub.get_instance(), gnss, _stuck[instance])) {
		return;
	}

	pub.publish(gnss);
}

Vector3f SensorGnssSim::antennaOffset(int instance) const
{
	if (instance == 0) {
		return {_param_gnss0_offx.get(), _param_gnss0_offy.get(), _param_gnss0_offz.get()};
	}

	return {_param_gnss1_offx.get(), _param_gnss1_offy.get(), _param_gnss1_offz.get()};
}

void SensorGnssSim::publishRelativeHeading(int instance, const sensor_gnss_s &gnss, const Dcmf &body_to_ned)
{
#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	static constexpr int32_t HEADING_MOVING_BASE_ROVER = 1;
	static constexpr int32_t HEADING_DUAL_ANTENNA = 2;

	const int32_t heading_setup = (instance == 0) ? _param_gnss0_hdg.get() : _param_gnss1_hdg.get();
	Vector3f baseline{};

	if (heading_setup == HEADING_MOVING_BASE_ROVER) {
		baseline = antennaOffset(instance) - antennaOffset(1 - instance);

	} else if (heading_setup == HEADING_DUAL_ANTENNA) {
		const Vector3f aux_antenna = (instance == 0)
					     ? Vector3f{_param_gnss0_auxx.get(), _param_gnss0_auxy.get(), _param_gnss0_auxz.get()}
					     : Vector3f{_param_gnss1_auxx.get(), _param_gnss1_auxy.get(), _param_gnss1_auxz.get()};
		baseline = aux_antenna - antennaOffset(instance);

	} else {
		return;
	}

	const float baseline_length = baseline.norm();

	if (baseline_length < FLT_EPSILON) {
		return;
	}

	const Vector3f relative_position = body_to_ned * baseline + noiseGauss3f(_baseline_noise, _baseline_noise,
					   _baseline_noise);
	const bool valid = gnss.fix_type >= sensor_gnss_s::FIX_TYPE_3D;

	sensor_gnss_relative_s relative{};
	relative.timestamp_sample = gnss.timestamp_sample;
	relative.device_id = gnss.device_id;
	relative.relative_position_valid = valid;
	relative.carrier_solution_fixed = valid;
	relative.gnss_fix_ok = valid;
	relative.heading_valid = valid;
	relative.moving_base_mode = (heading_setup == HEADING_MOVING_BASE_ROVER);
	relative_position.copyTo(relative.position);
	relative.position_accuracy[0] = _baseline_noise;
	relative.position_accuracy[1] = _baseline_noise;
	relative.position_accuracy[2] = _baseline_noise;
	relative.position_length = relative_position.norm();
	relative.accuracy_length = _baseline_noise;
	relative.heading = atan2f(relative_position(1), relative_position(0));
	relative.heading_accuracy = _baseline_noise / baseline_length;

	uORB::PublicationMulti<sensor_gnss_s> &gnss_pub = _sensor_gnss_pub[instance];
	relative.timestamp = hrt_absolute_time();

	// The heading fails with its receiver
	if (!failure_injection::process(_failure_config, failure_injection_s::FAILURE_UNIT_SENSOR_GPS,
					gnss_pub.get_instance(), relative, _stuck_relative[instance])) {
		return;
	}

	_sensor_gnss_relative_pub[instance].publish(relative);

#else
	(void)instance;
	(void)gnss;
	(void)body_to_ned;
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
}

void SensorGnssSim::updateFailureConfig()
{
	_failure_config.update();
}

bool SensorGnssSim::updateRtcmCorrections()
{
	rtcm_data_s msg;

	for (int instance = 0; instance < _rtcm_corrections_sub.size(); instance++) {
		while (_rtcm_corrections_sub[instance].update(&msg)) {
			_last_rtcm_time = math::max(_last_rtcm_time, msg.timestamp);
		}
	}

	return (_last_rtcm_time != 0) && (hrt_elapsed_time(&_last_rtcm_time) < RTCM_TIMEOUT);
}

int SensorGnssSim::task_spawn(int argc, char *argv[])
{
	SensorGnssSim *instance = new SensorGnssSim();

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

int SensorGnssSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorGnssSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_gnss_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_gnss_sim_main(int argc, char *argv[])
{
	return ModuleBase::main(SensorGnssSim::desc, argc, argv);
}
