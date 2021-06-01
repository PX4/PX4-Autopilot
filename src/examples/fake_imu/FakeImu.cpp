/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "FakeImu.hpp"

using namespace time_literals;

FakeImu::FakeImu() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_px4_accel(1310988), // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	_px4_gyro(1310988)   // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
{
	_sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());

	PX4_INFO("Rate %.3f, Interval: %d us", (double)_px4_gyro.get_max_rate_hz(), _sensor_interval_us);

	_px4_accel.set_range(2000.f); // don't care

	_px4_gyro.set_scale(math::radians(2000.f) / static_cast<float>(INT16_MAX - 1)); // 2000 degrees/second max
}

bool FakeImu::init()
{
	ScheduleOnInterval(_sensor_interval_us);
	return true;
}

void FakeImu::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = hrt_absolute_time();
	gyro.samples = roundf(IMU_RATE_HZ / (1e6 / _sensor_interval_us));
	gyro.dt = 1e6 / IMU_RATE_HZ;

	const double dt_s = 1 / IMU_RATE_HZ;

	const double x_f0 = 0.0;    //    0 Hz X frequency start
	const double x_f1 = 10.0;   //   10 Hz X frequency stop

	const double y_f0 = 0.0;    //   10 Hz Y frequency start
	const double y_f1 = 100.0;  // 1000 Hz Y frequency stop

	const double z_f0 = 0.0;    //  100 Hz Z frequency start
	const double z_f1 = 1000.0; // 1000 Hz Z frequency stop

	// amplitude
	static constexpr double A = (INT16_MAX - 1);

	if (_time_start_us == 0) {
		_time_start_us = gyro.timestamp_sample;
	}

	// 10 second sweep
	const double T = 10.0;

	const double timestamp_sample_s = static_cast<double>(gyro.timestamp_sample - _time_start_us) / 1e6;

	float x_freq = 0;
	float y_freq = 0;
	float z_freq = 0;

	for (int n = 0; n < gyro.samples; n++) {
		// timestamp_sample corresponds to last sample
		const double t = timestamp_sample_s - (gyro.samples - n - 1) * dt_s;

		// linear-frequency chirp, see https://en.wikipedia.org/wiki/Chirp
		const double x_F = x_f0 + (x_f1 - x_f0) * t / (2 * T);
		const double y_F = y_f0 + (y_f1 - y_f0) * t / (2 * T);
		const double z_F = z_f0 + (z_f1 - z_f0) * t / (2 * T);

		gyro.x[n] = roundf(A * sin(2 * M_PI * x_F * t));
		gyro.y[n] = roundf(A * sin(2 * M_PI * y_F * t));
		gyro.z[n] = roundf(A * sin(2 * M_PI * z_F * t));

		if (n == 0) {
			x_freq = (x_f1 - x_f0) * (t / T) + x_f0;
			y_freq = (y_f1 - y_f0) * (t / T) + y_f0;
			z_freq = (z_f1 - z_f0) * (t / T) + z_f0;

			_px4_accel.update(gyro.timestamp_sample, x_freq, y_freq, z_freq);
		}
	}

	_px4_gyro.updateFIFO(gyro);

#if defined(FAKE_IMU_FAKE_ESC_STATUS)

	// publish fake esc status at ~10 Hz
	if (hrt_elapsed_time(&_esc_status_pub.get().timestamp) > 100_ms) {
		auto &esc_status = _esc_status_pub.get();

		esc_status.esc_count = 3;

		// ESC 0 follow X axis RPM
		if (!(timestamp_sample_s > 1.5 && timestamp_sample_s < 2.0)) {
			// simulate drop out at 1.5 to 2 seconds
			esc_status.esc[0].timestamp = hrt_absolute_time();
			esc_status.esc[0].esc_rpm = x_freq * 60;
		}

		// ESC 1 follow Y axis RPM
		if (!(timestamp_sample_s > 2.5 && timestamp_sample_s < 3.0)) {
			// simulate drop out at 2.5 to 3 seconds
			esc_status.esc[1].timestamp = hrt_absolute_time();
			esc_status.esc[1].esc_rpm = y_freq * 60;
		}

		// ESC 2 follow Z axis RPM
		if (!(timestamp_sample_s > 3.5 && timestamp_sample_s < 4.0)) {
			// simulate drop out at 3.5 to 4 seconds
			esc_status.esc[2].timestamp = hrt_absolute_time();
			esc_status.esc[2].esc_rpm = z_freq * 60;
		}

		// simulate brief dropout of all data
		if (timestamp_sample_s > 5.5 && timestamp_sample_s < 5.6) {
			esc_status.esc[0].esc_rpm = 0;
			esc_status.esc[1].esc_rpm = 0;
			esc_status.esc[2].esc_rpm = 0;
		}

		esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.update();
	}

#endif // FAKE_IMU_FAKE_ESC_STATUS
}

int FakeImu::task_spawn(int argc, char *argv[])
{
	FakeImu *instance = new FakeImu();

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

int FakeImu::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeImu::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_imu", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_imu_main(int argc, char *argv[])
{
	return FakeImu::main(argc, argv);
}
