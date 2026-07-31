/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/events.h>
#include "navputer.hpp"

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t navputer_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<Navputer *> _instance {};

Navputer::Navputer(const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config)
{
	AdvertiseTopics();
}

Navputer::~Navputer()
{

}

void Navputer::AdvertiseTopics()
{
	_attitude_pub.advertise();
}


int Navputer::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

	Navputer *_inst = new Navputer(px4::wq_configurations::INS0, replay_mode);

	if (_inst) {
		_instance.store(_inst);
		_inst->ScheduleNow();
		success = true;
	}

	return success ? PX4_OK : PX4_ERROR;
}

int Navputer::print_status(bool verbose)
{
	PX4_INFO_RAW("Navputer is OK\n");

	return 0;
}

void Navputer::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	if (!_callback_registered) {
		_callback_registered = _sensor_combined_sub.registerCallback();

		if (!_callback_registered) {
			ScheduleDelayed(10_ms);
			return;
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

	sensor_combined_s sensor_combined;
	imu_updated = _sensor_combined_sub.update(&sensor_combined);

	if (imu_updated) {
		imu_sample_new.time_us = sensor_combined.timestamp;
		imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
		imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

		if (sensor_combined.accelerometer_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
		}

		imu_dt = sensor_combined.gyro_integral_dt;

		if (sensor_combined.accel_calibration_count != _accel_calibration_count) {

			PX4_DEBUG("%d - resetting accelerometer bias", _instance);

			_ekf.resetAccelBias();
			_accel_calibration_count = sensor_combined.accel_calibration_count;

			// reset bias learning
			_accel_cal = {};
		}

		if (sensor_combined.gyro_calibration_count != _gyro_calibration_count) {

			PX4_DEBUG("%d - resetting rate gyro bias", _instance);

			_ekf.resetGyroBias();
			_gyro_calibration_count = sensor_combined.gyro_calibration_count;

			// reset bias learning
			_gyro_cal = {};
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.airspeed_validated_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

		UpdateBaroSample(ekf2_timestamps);


		if (_ekf.update()) {

		}

		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void Navputer::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		navput_attitude_s att;
		att.timestamp_sample = timestamp;
		_ekf.getQuaternion().copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = hrt_absolute_time();
		_attitude_pub.publish(att);
	}
}

void Navputer::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {

		bool reset = false;

		// check if barometer has changed
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				PX4_DEBUG("baro sensor ID changed %" PRIu32 " -> %" PRIu32, _device_id_baro, airdata.baro_device_id);
			}

			reset = true;

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// existing calibration has changed, reset saved baro bias
			PX4_DEBUG("baro %" PRIu32 " calibration updated, resetting bias", _device_id_baro);
			reset = true;
		}

		if (reset) {
			_device_id_baro = airdata.baro_device_id;
			_baro_calibration_count = airdata.calibration_count;
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter, reset});

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

int Navputer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Navputer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	return 0;
}

extern "C" __EXPORT int navputer_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return Navputer::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		Navputer::lock_module();

		ret = Navputer::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		Navputer::unlock_module();
		return ret;
	} else if (strcmp(argv[1], "status") == 0) {
		if (Navputer::trylock_module()) {
			bool verbose_status = false;

			_instance.load()->print_status(verbose_status);

			Navputer::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		Navputer::lock_module();

		// otherwise stop everything
		bool was_running = false;

		Navputer *inst = _instance.load();

		if (inst) {
			PX4_INFO("stopping Navputer");
			was_running = true;
			inst->request_stop();
			px4_usleep(20000); // 20 ms
			delete inst;
			_instance.store(nullptr);
		}

		if (!was_running) {
			PX4_WARN("not running");
		}

		Navputer::unlock_module();
		return PX4_OK;
	}

	Navputer::lock_module(); // Lock here, as the method could access _instance.
	int ret = Navputer::custom_command(argc - 1, argv + 1);
	Navputer::unlock_module();

	return ret;
}
