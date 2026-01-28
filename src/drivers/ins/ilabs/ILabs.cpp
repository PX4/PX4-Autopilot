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

#include "ILabs.h"

#include <matrix/Euler.hpp>
#include <matrix/Quaternion.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#ifndef MODULE_NAME
#define MODULE_NAME "ilabs_ins_driver"  // NOLINT(cppcoreguidelines-macro-usage)
#endif

using namespace time_literals;

// GPS epoch: 1980-01-06 00:00:00 UTC
constexpr uint64_t GPS_EPOCH_SECS = 315964800ULL;

constexpr uint8_t DECIMATION_VALUE = 20;

uint64_t ToUtcMicroseconds(uint16_t gpsWeek, uint32_t msTow) {
	const uint64_t gpsTimeSec = gpsWeek * 7ULL * 86400ULL + msTow / 1000ULL;
	// 18 is a leap seconds correction
	const uint64_t utcTimeSec = gpsTimeSec + GPS_EPOCH_SECS - 18;
	const uint64_t timeUtcUsec = utcTimeSec * 1'000'000 + (msTow % 1000) * 1000ULL;
	return timeUtcUsec;
}

enum ILabsMode {
	RAW_SENSORS_DATA = 0,
	FULL_INS         = 1,
};

ILabs::ILabs(const char *serialDeviceName)
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serialDeviceName)),
	  _attitude_pub((_param_ilabs_mode.get() == ILabsMode::RAW_SENSORS_DATA) ? ORB_ID(external_ins_attitude)
																			 : ORB_ID(vehicle_attitude)),
	  _local_position_pub((_param_ilabs_mode.get() == ILabsMode::RAW_SENSORS_DATA) ? ORB_ID(external_ins_local_position)
																				   : ORB_ID(vehicle_local_position)),
	  _global_position_pub((_param_ilabs_mode.get() == ILabsMode::RAW_SENSORS_DATA)
							   ? ORB_ID(external_ins_global_position)
							   : ORB_ID(vehicle_global_position)) {
	// store port name
	strncpy(_serialDeviceName, serialDeviceName, sizeof(_serialDeviceName) - 1);

	// enforce null termination
	_serialDeviceName[sizeof(_serialDeviceName) - 1] = '\0';

	if (_param_ilabs_mode.get() == ILabsMode::FULL_INS) {
		int32_t value = 0;
		param_set(param_find("SENS_IMU_MODE"), &value);
		param_set(param_find("SENS_MAG_MODE"), &value);
	}

	_device_id.devid_s.devtype  = DRV_INS_DEVTYPE_ILABS;
	_device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	_px4_accel.set_device_id(_device_id.devid);
	_px4_gyro.set_device_id(_device_id.devid);
	_px4_mag.set_device_id(_device_id.devid);

	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
	_sensor_baro_pub.advertise();
	_sensor_gps_pub.advertise();
}

ILabs::~ILabs() {
	_sensor.deinit();

	perf_free(_sample_perf);
	perf_free(_comms_errors);

	perf_free(_accel_pub_interval_perf);
	perf_free(_gyro_pub_interval_perf);
	perf_free(_mag_pub_interval_perf);
	perf_free(_gnss_pub_interval_perf);
	perf_free(_baro_pub_interval_perf);

	perf_free(_attitude_pub_interval_perf);
	perf_free(_local_position_pub_interval_perf);
	perf_free(_global_position_pub_interval_perf);
}

int ILabs::task_spawn(int argc, char *argv[]) {
	bool error_flag = false;

	int         opt_index   = 1;
	const char *opt_arg     = nullptr;
	int         opt_val     = 0;
	const char *device_name = nullptr;

	while ((opt_val = px4_getopt(argc, argv, "d:", &opt_index, &opt_arg)) != EOF) {
		switch (opt_val) {
			case 'd':
				device_name = opt_arg;
				break;

			case '?':
				error_flag = true;
				break;

			default:
				PX4_WARN("Unrecognized flag");
				error_flag = true;
				break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		ILabs *instance = new ILabs(device_name);

		if (instance == nullptr) {
			PX4_ERR("Alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();

		return PX4_OK;
	}

	if (device_name) {
		PX4_ERR("Invalid device (-d) %s", device_name);

	} else {
		PX4_INFO("Valid device required");
	}

	return PX4_ERROR;
}

int ILabs::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int ILabs::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the ILabs sensors.

Most boards are configured to enable/start the driver on a specified UART using the SENS_ILABS_CFG.
After that you can use the ILABS_MODE parameter to config outputs:

- Only raw sensor output (the default).
- Sensor output and INS data such as position and velocity estimates.

Setup/usage information: https://docs.px4.io/main/en/sensor/inertiallabs.html


### Examples

Attempt to start driver on a specified serial device.
$ ilabs start -d /dev/ttyS1
Stop driver
$ ilabs stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ilabs", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");

	return PX4_OK;
}

int ILabs::print_status() {
	if (_serialDeviceName[0] != '\0') {
		PX4_INFO("UART device: %s", _serialDeviceName);
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

void ILabs::Run() {
	if (should_exit()) {
		_sensor.deinit();
		exit_and_cleanup();
		return;
	}

	if (!_sensor.isInitialized()) {
		const bool result = _sensor.init(_serialDeviceName, this, &ILabs::processDataProxy);

		if (!result) {
			PX4_ERR("Sensor initializing error");
			_sensor.deinit();
			_time_initialized.store(0);
			_time_last_valid_imu_data.store(0);
			ScheduleDelayed(3_s);
			return;
		}
		_time_initialized.store(hrt_absolute_time());
	}

	const hrt_abstime time_initialized         = _time_initialized.load();
	const hrt_abstime time_last_valid_imu_data = _time_last_valid_imu_data.load();

	// Update sensor_selection for FULL_INS mode
	if (_param_ilabs_mode.get() == ILabsMode::FULL_INS && time_initialized != 0 && time_last_valid_imu_data != 0 &&
	    hrt_elapsed_time(&time_last_valid_imu_data) < 3_s) {

		if ((_px4_accel.get_device_id() != 0) && (_px4_gyro.get_device_id() != 0)) {
			sensor_selection_s sensor_selection{};
			sensor_selection.accel_device_id = _px4_accel.get_device_id();
			sensor_selection.gyro_device_id  = _px4_gyro.get_device_id();
			sensor_selection.timestamp       = time_initialized;
			_sensor_selection_pub.publish(sensor_selection);
		} else {
			PX4_ERR("Sensor not initialized");
		}
	}

	// Missing data handling
	if (time_initialized != 0 && time_last_valid_imu_data != 0 &&
	    hrt_elapsed_time(&time_last_valid_imu_data) > 3_s) {
		PX4_ERR("Timeout: no new data from sensor. Reinitializing");
		_sensor.deinit();
		_time_initialized.store(0);
		_time_last_valid_imu_data.store(0);
		ScheduleDelayed(3_s);
		return;
	}

	ScheduleDelayed(100_ms);
}

void ILabs::processData(InertialLabs::SensorsData *data) {
	// PX4 by default uses FRD/NED frame position
	// Inertial Labs INS by default uses in RFU/ENU frame position

	if (!data) {
		PX4_ERR("Invalid sensor data");
		return;
	}

	const bool isFilterOk = (data->ins.unitStatus & InertialLabs::USW::INITIAL_ALIGNMENT_FAIL) == 0 &&
				 data->ins.solutionStatus != InertialLabs::InsSolution::INVALID;
	const bool isAccelOk = (data->ins.unitStatus & InertialLabs::USW::ACCEL_FAIL) == 0;
	const bool isGyroOk  = (data->ins.unitStatus & InertialLabs::USW::GYRO_FAIL) == 0;
	const bool isMagOk   = (data->ins.unitStatus & InertialLabs::USW::MAG_FAIL) == 0;

	// true if received new GNSS position or velocity
	const bool hasNewGpsData = (data->gps.newData & (InertialLabs::NewGpsData::NEW_GNSS_POSITION | InertialLabs::NewGpsData::NEW_GNSS_VELOCITY));
	const bool hasEnoughSatellites = data->gps.usedSatCount > 3;

	const bool isBaroOk = (data->ins.unitStatus2 & InertialLabs::USW2::ADU_BARO_FAIL) == 0;

	// TODO wind data:
	// const bool isDiffPressureOk = (data->ins.unitStatus2 & InertialLabs::USW2::ADU_DIFF_PRESS_FAIL) == 0;

	const hrt_abstime time_now_us = hrt_absolute_time();
	_time_last_valid_imu_data.store(time_now_us);

	// update all temperatures
	if (isFilterOk) {
		_px4_accel.set_temperature(data->temperature);  // degC
		_px4_gyro.set_temperature(data->temperature);   // degC
		_px4_mag.set_temperature(data->temperature);    // degC
	}

	// publish accel
	if (isFilterOk && isAccelOk) {
		_px4_accel.update(time_now_us, data->accel(0), data->accel(1), data->accel(2));  // NED, in m/s^2
		perf_count(_accel_pub_interval_perf);
	}

	// publish gyro
	if (isFilterOk && isGyroOk) {
		_px4_gyro.update(time_now_us, data->gyro(0), data->gyro(1), data->gyro(2));  // NED, in rad/s
		perf_count(_gyro_pub_interval_perf);
	}

	// publish mag
	if (isFilterOk && isMagOk) {
		_px4_mag.update(time_now_us, data->mag(0), data->mag(1), data->mag(2));  // NED, in Gauss
		perf_count(_mag_pub_interval_perf);
	}

	// publish baro
	if (isFilterOk && isBaroOk) {
		if (_average_sensors_data.count > DECIMATION_VALUE) {
			sensor_baro_s sensor_baro{};
			sensor_baro.timestamp = time_now_us;
			sensor_baro.timestamp_sample = time_now_us;

			sensor_baro.device_id   = _device_id.devid;
			sensor_baro.pressure    = _average_sensors_data.pressure / static_cast<float>(_average_sensors_data.count);    // Pa
			sensor_baro.temperature = _average_sensors_data.temperature / static_cast<float>(_average_sensors_data.count);  // degC

			_sensor_baro_pub.publish(sensor_baro);
			perf_count(_baro_pub_interval_perf);

			_average_sensors_data.count = 0;
			_average_sensors_data.pressure = 0.0f;
			_average_sensors_data.temperature = 0.0f;
		}

		_average_sensors_data.pressure += data->pressure;  // Pa
		_average_sensors_data.temperature += data->temperature;  // degC
		++_average_sensors_data.count;
	}

	// publish attitude
	if (isFilterOk) {
		// TODO: Use quaternion message in UDD?
		const matrix::Quatf quat{matrix::Eulerf(math::radians(data->ins.roll),
							math::radians(data->ins.pitch),
							math::radians(data->ins.yaw))};
		vehicle_attitude_s  attitude{};
		attitude.timestamp        = time_now_us;
		attitude.timestamp_sample = time_now_us;

		attitude.q[0] = quat(0);
		attitude.q[1] = quat(1);
		attitude.q[2] = quat(2);
		attitude.q[3] = quat(3);

		_attitude_pub.publish(attitude);
		perf_count(_attitude_pub_interval_perf);
	}

	// publish local position
	if (isFilterOk) {
		vehicle_local_position_s local_position{};
		local_position.timestamp        = time_now_us;
		local_position.timestamp_sample = time_now_us;

		local_position.xy_valid   = true;
		local_position.z_valid    = true;
		local_position.v_xy_valid = true;
		local_position.v_z_valid  = true;

		if (!_pos_ref.isInitialized()) {
			_pos_ref.initReference(data->ins.latitude, data->ins.longitude, time_now_us);
		}
		const matrix::Vector2f pos_ned = _pos_ref.project(data->ins.latitude, data->ins.longitude);
		local_position.x               = pos_ned(0);
		local_position.y               = pos_ned(1);
		local_position.z               = data->ins.altitude;

		local_position.vx = data->ins.velocity(0);
		local_position.vy = data->ins.velocity(1);
		local_position.vz = data->ins.velocity(2);

		local_position.ax = data->accel(0);
		local_position.ay = data->accel(1);
		local_position.az = data->accel(2);

		local_position.heading = static_cast<float>(data->ext.headingData.heading) *
								 static_cast<float>(M_DEG_TO_RAD) * 0.01f;  // rad
		local_position.unaided_heading          = NAN;
		local_position.heading_good_for_control = true;

		local_position.xy_global     = true;
		local_position.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
		local_position.ref_lat       = _pos_ref.getProjectionReferenceLat();
		local_position.ref_lon       = _pos_ref.getProjectionReferenceLon();
		local_position.z_global      = true;

		local_position.dist_bottom_valid = false;

		const float lat_err = static_cast<float>(data->ins.accuracy.lat) * 0.001f;
		const float lon_err = static_cast<float>(data->ins.accuracy.lon) * 0.001f;
		local_position.eph = sqrtf(lat_err * lat_err + lon_err * lon_err);
		local_position.epv = static_cast<float>(data->ins.accuracy.alt) * 0.001f;

		const float northVel_err = static_cast<float>(data->ins.accuracy.northVel) * 0.001f;
		const float eastVel_err = static_cast<float>(data->ins.accuracy.eastVel) * 0.001f;
		local_position.evh = sqrtf(northVel_err * northVel_err + eastVel_err * eastVel_err);

		local_position.evv = static_cast<float>(data->ins.accuracy.verVel) * 0.001f;

		local_position.dead_reckoning = false;

		local_position.vxy_max     = INFINITY;
		local_position.vz_max      = INFINITY;
		local_position.hagl_min    = INFINITY;
		local_position.hagl_max_z  = INFINITY;
		local_position.hagl_max_xy = INFINITY;

		_local_position_pub.publish(local_position);
		perf_count(_local_position_pub_interval_perf);
	}

	// publish global_position
	if (isFilterOk) {
		vehicle_global_position_s global_position{};
		global_position.timestamp        = time_now_us;
		global_position.timestamp_sample = time_now_us;

		global_position.lat_lon_valid = true;
		global_position.alt_valid     = true;
		global_position.lat           = data->ins.latitude;
		global_position.lon           = data->ins.longitude;
		global_position.alt           = data->ins.altitude;

		const float lat_err = static_cast<float>(data->ins.accuracy.lat) * 0.001f;
		const float lon_err = static_cast<float>(data->ins.accuracy.lon) * 0.001f;
		global_position.eph           = sqrtf(lat_err * lat_err + lon_err * lon_err);
		global_position.epv           = static_cast<float>(data->ins.accuracy.alt) * 0.001f;

		global_position.dead_reckoning = false;

		_global_position_pub.publish(global_position);
		perf_count(_global_position_pub_interval_perf);
	}

	// publish GPS data
	if (hasEnoughSatellites && isFilterOk && hasNewGpsData) {
		sensor_gps_s sensor_gps{};
		sensor_gps.timestamp        = time_now_us;
		sensor_gps.timestamp_sample = time_now_us;

		sensor_gps.device_id = _device_id.devid;

		sensor_gps.latitude_deg   = data->gps.latitude;
		sensor_gps.longitude_deg  = data->gps.longitude;
		sensor_gps.altitude_ellipsoid_m = data->gps.altitude;
		sensor_gps.altitude_msl_m = data->gps.altitude;

		sensor_gps.fix_type = data->gps.fixType + 1;

		const float lat_err = static_cast<float>(data->ins.accuracy.lat) * 0.001f;
		const float lon_err = static_cast<float>(data->ins.accuracy.lon) * 0.001f;
		sensor_gps.eph = sqrtf(lat_err * lat_err + lon_err * lon_err);
		sensor_gps.epv = static_cast<float>(data->ins.accuracy.alt) * 0.001f;

		sensor_gps.hdop = static_cast<float>(data->gps.dop.hdop) * 0.001f;
		sensor_gps.vdop = static_cast<float>(data->gps.dop.vdop) * 0.001f;

		sensor_gps.jamming_state = data->gps.jamStatus;
		sensor_gps.jamming_indicator = (sensor_gps.jamming_state != InertialLabs::JammingStatus::UNKOWN_OR_DISABLED) &&
					       (sensor_gps.jamming_state != InertialLabs::JammingStatus::OK);
		sensor_gps.spoofing_state = data->gps.spoofingStatus;

		sensor_gps.vel_m_s =
			matrix::Vector3f(data->ins.velocity(0), data->ins.velocity(1), data->ins.velocity(2)).length();
		sensor_gps.vel_n_m_s     = data->ins.velocity(0);
		sensor_gps.vel_e_m_s     = data->ins.velocity(1);
		sensor_gps.vel_d_m_s     = data->ins.velocity(2);
		sensor_gps.vel_ned_valid = true;

		sensor_gps.time_utc_usec = ToUtcMicroseconds(data->gps.gpsWeek, data->gps.msTow);
		sensor_gps.timestamp_time_relative = static_cast<int32_t>(sensor_gps.time_utc_usec - time_now_us);

		sensor_gps.satellites_used = data->gps.usedSatCount;

		sensor_gps.heading = NAN;
		sensor_gps.heading_offset = NAN;
		sensor_gps.heading_accuracy = NAN;

		// sensor_gps.s_variance_m_s = ...; // TODO: need 0x43 UDD Package?

		_sensor_gps_pub.publish(sensor_gps);
		perf_count(_gnss_pub_interval_perf);
	}

	if (_param_ilabs_mode.get() == ILabsMode::FULL_INS) {
		estimator_status_s estimator_status{};
		estimator_status.timestamp        = time_now_us;
		estimator_status.timestamp_sample = time_now_us;

		const float test_ratio = 0.1f;

		estimator_status.hdg_test_ratio = test_ratio;
		estimator_status.vel_test_ratio = test_ratio;
		estimator_status.pos_test_ratio = test_ratio;
		estimator_status.hgt_test_ratio = test_ratio;

		estimator_status.accel_device_id = _px4_accel.get_device_id();
		estimator_status.gyro_device_id  = _px4_gyro.get_device_id();
		estimator_status.mag_device_id  = _device_id.devid;
		estimator_status.baro_device_id  = _device_id.devid;

		_estimator_status_pub.publish(estimator_status);
	}
}

extern "C" __EXPORT int ilabs_main(int argc, char *argv[]) {
	return ILabs::main(argc, argv);
}
