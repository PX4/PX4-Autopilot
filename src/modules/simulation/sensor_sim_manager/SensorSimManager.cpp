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

#include "SensorSimManager.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using matrix::sign;

using namespace matrix;

SensorSimManager::SensorSimManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_gen(_rd()),
	_uniform_dist(0.0f, 1.0f)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_MAGSIM);
	srand(1234);

	// Initialize sensor timings with base intervals and random offsets
	const hrt_abstime now = hrt_absolute_time();

	// GPS: 8 Hz
	_gps_timing = {
		.interval_us = 125000,
		.next_update_time = now,
		.offset_us = static_cast<hrt_abstime>(_uniform_dist(_gen) * 125000),
		.enabled = false,
	};

	// Barometer: 20 Hz
	_baro_timing = {
		.interval_us = 50000,
		.next_update_time = now,
		.offset_us = static_cast<hrt_abstime>(_uniform_dist(_gen) * 50000),
		.enabled = false,
	};

	// Magnetometer: 50 Hz
	_mag_timing = {
		.interval_us = 20000,
		.next_update_time = now,
		.offset_us = static_cast<hrt_abstime>(_uniform_dist(_gen) * 20000),
		.enabled = false,
	};

	// Airspeed: 8 Hz
	_airspeed_timing = {
		.interval_us = 125000,
		.next_update_time = now,
		.offset_us = static_cast<hrt_abstime>(_uniform_dist(_gen) * 125000),
		.enabled = false,
	};

	// AGP: 2 Hz
	_agp_timing = {
		.interval_us = 500000,
		.next_update_time = now,
		.offset_us = static_cast<hrt_abstime>(_uniform_dist(_gen) * 500000),
		.enabled = false,
	};

	_gps_timing.next_update_time += _gps_timing.offset_us;
	_baro_timing.next_update_time += _baro_timing.offset_us;
	_mag_timing.next_update_time += _mag_timing.offset_us;
	_airspeed_timing.next_update_time += _airspeed_timing.offset_us;
	_agp_timing.next_update_time += _agp_timing.offset_us;

}

SensorSimManager::~SensorSimManager()
{
	perf_free(_loop_perf);
	perf_free(_gps_perf);
	perf_free(_baro_perf);
	perf_free(_mag_perf);
	perf_free(_airspeed_perf);
	perf_free(_agp_perf);
}

bool SensorSimManager::init()
{
	ScheduleOnInterval(10_ms);
	return true;
}

float SensorSimManager::generate_wgn()
{
	// Generate white Gaussian noise sample with std=1
	// Using Box-Muller transform
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

matrix::Vector3f SensorSimManager::noiseGauss3f(float stdx, float stdy, float stdz)
{
	return matrix::Vector3f(generate_wgn() * stdx, generate_wgn() * stdy, generate_wgn() * stdz);
}

void SensorSimManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();

		_gps_timing.enabled = (_param_sens_en_gpssim.get() != 0);
		_baro_timing.enabled = (_param_sens_en_barosim.get() != 0);
		_mag_timing.enabled = (_param_sens_en_magsim.get() != 0);
		_airspeed_timing.enabled = (_param_sens_en_arspdsim.get() != 0);
		_agp_timing.enabled = (_param_sens_en_agpsim.get() != 0);
	}

	_failure_injection.check_failure_injections();

	const hrt_abstime now = hrt_absolute_time();

	if (_gps_timing.enabled && now >= _gps_timing.next_update_time) {
		updateGPS();
		_gps_timing.next_update_time = now + _gps_timing.interval_us;
	}

	if (_baro_timing.enabled && now >= _baro_timing.next_update_time) {
		updateBarometer();
		_baro_timing.next_update_time = now + _baro_timing.interval_us;
	}

	if (_mag_timing.enabled && now >= _mag_timing.next_update_time) {
		updateMagnetometer();
		_mag_timing.next_update_time = now + _mag_timing.interval_us;
	}

	if (_airspeed_timing.enabled && now >= _airspeed_timing.next_update_time) {
		updateAirspeed();
		_airspeed_timing.next_update_time = now + _airspeed_timing.interval_us;
	}

	if (_agp_timing.enabled && now >= _agp_timing.next_update_time) {
		updateAGP();
		_agp_timing.next_update_time = now + _agp_timing.interval_us;
	}

	perf_end(_loop_perf);
}

void SensorSimManager::updateGPS()
{
	perf_begin(_gps_perf);

	_failure_injection.check_failure_injections();

	vehicle_local_position_s lpos{};
	_vehicle_local_position_sub.copy(&lpos);

	vehicle_global_position_s gpos{};
	_vehicle_global_position_sub.copy(&gpos);

	// Check if data is recent (within 20ms), TODO
	const hrt_abstime now = hrt_absolute_time();

	if (lpos.timestamp > 0 && gpos.timestamp > 0 &&
	    (now - lpos.timestamp) < 20000 && (now - gpos.timestamp) < 20000) {

		double latitude = gpos.lat + math::degrees((double)generate_wgn() * 0.2 / CONSTANTS_RADIUS_OF_EARTH);
		double longitude = gpos.lon + math::degrees((double)generate_wgn() * 0.2 / CONSTANTS_RADIUS_OF_EARTH);
		double altitude = (double)(gpos.alt + (generate_wgn() * 0.5f));

		Vector3f gps_vel = Vector3f{lpos.vx, lpos.vy, lpos.vz} + noiseGauss3f(0.06f, 0.077f, 0.158f);

		// device id
		device::Device::DeviceId device_id;
		device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
		device_id.devid_s.bus = 0;
		device_id.devid_s.address = 0;
		device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

		sensor_gps_s sensor_gps{};

		if (_sim_gps_used.get() >= 4) {
			// fix
			sensor_gps.fix_type = 3; // 3D fix
			sensor_gps.s_variance_m_s = 0.4f;
			sensor_gps.c_variance_rad = 0.1f;
			sensor_gps.eph = 0.9f;
			sensor_gps.epv = 1.78f;
			sensor_gps.hdop = 0.7f;
			sensor_gps.vdop = 1.1f;

		} else {
			// no fix
			sensor_gps.fix_type = 0; // No fix
			sensor_gps.s_variance_m_s = 100.f;
			sensor_gps.c_variance_rad = 100.f;
			sensor_gps.eph = 100.f;
			sensor_gps.epv = 100.f;
			sensor_gps.hdop = 100.f;
			sensor_gps.vdop = 100.f;
		}

		sensor_gps.timestamp_sample = gpos.timestamp_sample;
		sensor_gps.time_utc_usec = 0;
		sensor_gps.device_id = device_id.devid;
		sensor_gps.latitude_deg = latitude; // Latitude in degrees
		sensor_gps.longitude_deg = longitude; // Longitude in degrees
		sensor_gps.altitude_msl_m = altitude; // Altitude in meters above MSL
		sensor_gps.altitude_ellipsoid_m = altitude;
		sensor_gps.noise_per_ms = 0;
		sensor_gps.jamming_indicator = 0;
		sensor_gps.vel_m_s = sqrtf(gps_vel(0) * gps_vel(0) + gps_vel(1) * gps_vel(1)); // GPS ground speed, (metres/sec)
		sensor_gps.vel_n_m_s = gps_vel(0);
		sensor_gps.vel_e_m_s = gps_vel(1);
		sensor_gps.vel_d_m_s = gps_vel(2);
		sensor_gps.cog_rad = atan2(gps_vel(1),
					   gps_vel(0)); // Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
		sensor_gps.timestamp_time_relative = 0;
		sensor_gps.heading = NAN;
		sensor_gps.heading_offset = NAN;
		sensor_gps.heading_accuracy = 0;
		sensor_gps.automatic_gain_control = 0;
		sensor_gps.jamming_state = 0;
		sensor_gps.spoofing_state = 0;
		sensor_gps.vel_ned_valid = true;
		sensor_gps.satellites_used = _sim_gps_used.get();

		sensor_gps.timestamp = hrt_absolute_time();

		// Apply failure injection
		if (_failure_injection.handle_gps_failure(sensor_gps) && _failure_injection.handle_gps_alt_failure(sensor_gps)) {
			_sensor_gps_pub.publish(sensor_gps);

		}
	}

	perf_end(_gps_perf);
}

void SensorSimManager::updateBarometer()
{
	perf_begin(_baro_perf);

	vehicle_global_position_s gpos;

	if (_vehicle_global_position_sub.copy(&gpos)) {
		// Check if data is recent (within 20ms), TODO
		const hrt_abstime now = hrt_absolute_time();

		if (gpos.timestamp > 0 && (now - gpos.timestamp) < 20000) {
			const float dt = math::constrain((gpos.timestamp - _last_baro_update_time) * 1e-6f, 0.001f, 0.1f);

			const float alt_msl = gpos.alt;

			const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_msl;
			const float pressure_ratio = powf(TEMPERATURE_MSL / temperature_local, 5.256f);
			const float absolute_pressure = PRESSURE_MSL / pressure_ratio;

			// generate Gaussian noise sequence using polar form of Box-Muller transformation
			double y1;
			{
				double x1;
				double x2;
				double w;

				if (!_baro_rnd_use_last) {
					do {
						x1 = 2. * (double)generate_wgn() - 1.;
						x2 = 2. * (double)generate_wgn() - 1.;
						w = x1 * x1 + x2 * x2;
					} while (w >= 1.0);

					w = sqrt((-2.0 * log(w)) / w);
					// calculate two values - the second value can be used next time because it is uncorrelated
					y1 = x1 * w;
					_baro_rnd_y2 = x2 * w;
					_baro_rnd_use_last = true;

				} else {
					// no need to repeat the calculation - use the second value from last update
					y1 = _baro_rnd_y2;
					_baro_rnd_use_last = false;
				}
			}

			// Apply noise and drift
			const float abs_pressure_noise = 1.f * (float)y1;  // 1 Pa RMS noise
			_baro_drift_pa += _baro_drift_pa_per_sec * dt;
			const float absolute_pressure_noisy = absolute_pressure + abs_pressure_noise + _baro_drift_pa;

			// convert to hPa
			float pressure = absolute_pressure_noisy + _sim_baro_off_p.get();

			// calculate temperature in Celsius
			float temperature = temperature_local - 273.0f + _sim_baro_off_t.get();

			// publish
			sensor_baro_s sensor_baro{};
			sensor_baro.timestamp_sample = gpos.timestamp;
			sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
			sensor_baro.pressure = pressure;
			sensor_baro.temperature = temperature;
			sensor_baro.timestamp = hrt_absolute_time();
			_sensor_baro_pub.publish(sensor_baro);

			_last_baro_update_time = gpos.timestamp;
		}
	}

	perf_end(_baro_perf);
}

void SensorSimManager::updateMagnetometer()
{
	perf_begin(_mag_perf);

	vehicle_global_position_s gpos;

	if (_vehicle_global_position_sub.copy(&gpos)) {
		// Check if data is recent (within 20ms), TODO
		const hrt_abstime now = hrt_absolute_time();

		if (gpos.timestamp > 0 && (now - gpos.timestamp) < 20000 && gpos.eph < 1000) {
			// magnetic field data returned by the geo library using the current GPS position
			const float declination_rad = math::radians(get_mag_declination_degrees(gpos.lat, gpos.lon));
			const float inclination_rad = math::radians(get_mag_inclination_degrees(gpos.lat, gpos.lon));
			const float field_strength_gauss = get_mag_strength_gauss(gpos.lat, gpos.lon);

			_mag_earth_pred = Dcmf(Eulerf(0, -inclination_rad, declination_rad)) * Vector3f(field_strength_gauss, 0, 0);

			_mag_earth_available = true;
		}
	}

	if (_mag_earth_available) {
		vehicle_attitude_s attitude;

		if (_vehicle_attitude_sub.copy(&attitude)) {
			Vector3f expected_field = Dcmf{Quatf{attitude.q}} .transpose() * _mag_earth_pred;

			expected_field += noiseGauss3f(0.02f, 0.02f, 0.03f);

			_px4_mag.update(attitude.timestamp,
					expected_field(0) + _sim_mag_offset_x.get(),
					expected_field(1) + _sim_mag_offset_y.get(),
					expected_field(2) + _sim_mag_offset_z.get());
		}
	}

	perf_end(_mag_perf);
}

void SensorSimManager::updateAirspeed()
{
	perf_begin(_airspeed_perf);

	vehicle_local_position_s lpos{};
	_vehicle_local_position_sub.copy(&lpos);

	vehicle_global_position_s gpos{};
	_vehicle_global_position_sub.copy(&gpos);

	vehicle_attitude_s attitude{};
	_vehicle_attitude_sub.copy(&attitude);

	// Check if data is recent (within 20ms), TODO
	const hrt_abstime now = hrt_absolute_time();

	if (lpos.timestamp > 0 && gpos.timestamp > 0 && attitude.timestamp > 0 &&
	    (now - lpos.timestamp) < 20000 && (now - gpos.timestamp) < 20000 && (now - attitude.timestamp) < 20000) {

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
		differential_pressure.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		differential_pressure.differential_pressure_pa = (double)diff_pressure * 100.0; // hPa to Pa;
		differential_pressure.temperature = temperature_local;
		differential_pressure.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(differential_pressure);
	}

	perf_end(_airspeed_perf);
}

void SensorSimManager::updateAGP()
{
	perf_begin(_agp_perf);

	vehicle_global_position_s gpos{};
	_vehicle_global_position_sub.copy(&gpos);

	// Check if data is recent (within 20ms), TODO
	const hrt_abstime now = hrt_absolute_time();

	if (gpos.timestamp > 0 && (now - gpos.timestamp) < 20000) {

		const uint64_t current_time = gpos.timestamp;
		const float dt = (current_time - _agp_time_last_update) * 1e-6f;
		_agp_time_last_update = current_time;

		// Handle failure modes
		enum class FailureMode : int32_t {
			Stuck = (1 << 0),
			Drift = (1 << 1)
		};

		if (!(_sim_agp_fail.get() & static_cast<int32_t>(FailureMode::Stuck))) {
			_agp_measured_lla = LatLonAlt(gpos.lat, gpos.lon, gpos.alt_ellipsoid);
		}

		if (_sim_agp_fail.get() & static_cast<int32_t>(FailureMode::Drift)) {
			_agp_position_bias += matrix::Vector3f(1.5f, -5.f, 0.f) * dt;
			_agp_measured_lla += _agp_position_bias;

		} else {
			_agp_position_bias.zero();
		}

		const double latitude = _agp_measured_lla.latitude_deg() + math::degrees((double)generate_wgn() * 2.0 /
					CONSTANTS_RADIUS_OF_EARTH);
		const double longitude = _agp_measured_lla.longitude_deg() + math::degrees((double)generate_wgn() * 2.0 /
					 CONSTANTS_RADIUS_OF_EARTH);
		const double altitude = (double)(_agp_measured_lla.altitude() + (generate_wgn() * 0.5f));

		vehicle_global_position_s agp_sample{};

		agp_sample.timestamp_sample = gpos.timestamp_sample;
		agp_sample.lat = latitude;
		agp_sample.lon = longitude;
		agp_sample.alt = altitude;
		agp_sample.lat_lon_valid = true;
		agp_sample.alt_ellipsoid = altitude;
		agp_sample.alt_valid = true;
		agp_sample.eph = 20.f;
		agp_sample.epv = 5.f;

		agp_sample.timestamp = hrt_absolute_time();
		_aux_global_position_pub.publish(agp_sample);
	}

	perf_end(_agp_perf);
}

int SensorSimManager::task_spawn(int argc, char *argv[])
{
	SensorSimManager *instance = new SensorSimManager();

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

int SensorSimManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorSimManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Unified sensor simulation manager for SIH (Simulation in Hardware).
Simulates GPS, barometer, magnetometer, airspeed, and AGP sensors with realistic noise and timing.
Integrates failure injection capabilities for robust testing.

### Examples
To start the sensor simulation manager:
$ sensor_sim_manager start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_sim_manager", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_sim_manager_main(int argc, char *argv[])
{
	return SensorSimManager::main(argc, argv);
}
