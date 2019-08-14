/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_mpu6050_wrapper.cpp
 * Lightweight driver to access the MPU6050 of the DriverFramework.
 */

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>

#include <mpu6050/MPU6050.hpp>
#include <DevMgr.hpp>

extern "C" { __EXPORT int df_mpu6050_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMPU6050Wrapper : public MPU6050
{
public:
	DfMPU6050Wrapper(enum Rotation rotation);
	~DfMPU6050Wrapper();

	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

	/**
	 * Print some debug info.
	 */
	void		info();

private:
	int _publish(struct imu_sensor_data &data);

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	perf_counter_t		    _read_counter;
	perf_counter_t		    _error_counter;
	perf_counter_t		    _fifo_overflow_counter;
	perf_counter_t		    _fifo_corruption_counter;
	perf_counter_t		    _gyro_range_hit_counter;
	perf_counter_t		    _accel_range_hit_counter;
	perf_counter_t		    _publish_perf;

};

DfMPU6050Wrapper::DfMPU6050Wrapper(enum Rotation rotation) :
	MPU6050(IMU_DEVICE_PATH),
	_px4_accel(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_px4_gyro(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_read_counter(perf_alloc(PC_COUNT, "mpu6050_reads")),
	_error_counter(perf_alloc(PC_COUNT, "mpu6050_errors")),
	_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu6050_fifo_overflows")),
	_fifo_corruption_counter(perf_alloc(PC_COUNT, "mpu6050_fifo_corruptions")),
	_gyro_range_hit_counter(perf_alloc(PC_COUNT, "mpu6050_gyro_range_hits")),
	_accel_range_hit_counter(perf_alloc(PC_COUNT, "mpu6050_accel_range_hits")),
	_publish_perf(perf_alloc(PC_ELAPSED, "mpu6050_publish"))
{
	_px4_accel.set_scale(1.0f / 1000.0f);
	_px4_gyro.set_scale(1.0f / 1000.0f);
}

DfMPU6050Wrapper::~DfMPU6050Wrapper()
{
	perf_free(_read_counter);
	perf_free(_error_counter);
	perf_free(_fifo_overflow_counter);
	perf_free(_fifo_corruption_counter);
	perf_free(_gyro_range_hit_counter);
	perf_free(_accel_range_hit_counter);

	perf_free(_publish_perf);
}

int DfMPU6050Wrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("init fail: %d", ret);
		return ret;
	}

	ret = MPU6050::start();

	if (ret != 0) {
		PX4_ERR("start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfMPU6050Wrapper::stop()
{
	/* Stop sensor. */
	int ret = MPU6050::stop();

	if (ret != 0) {
		PX4_ERR("stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfMPU6050Wrapper::info()
{
	perf_print_counter(_read_counter);
	perf_print_counter(_error_counter);
	perf_print_counter(_fifo_overflow_counter);
	perf_print_counter(_fifo_corruption_counter);
	perf_print_counter(_gyro_range_hit_counter);
	perf_print_counter(_accel_range_hit_counter);
	perf_print_counter(_publish_perf);
}

int DfMPU6050Wrapper::_publish(struct imu_sensor_data &data)
{
	perf_begin(_publish_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Update all the counters.
	perf_set_count(_read_counter, data.read_counter);
	perf_set_count(_error_counter, data.error_counter);
	perf_set_count(_fifo_overflow_counter, data.fifo_overflow_counter);
	perf_set_count(_fifo_corruption_counter, data.fifo_overflow_counter);
	perf_set_count(_gyro_range_hit_counter, data.gyro_range_hit_counter);
	perf_set_count(_accel_range_hit_counter, data.accel_range_hit_counter);

	// MPU6050 driver from DriverFramework does not provide any raw values
	// TEMP We misuse the raw values on the Snapdragon to publish unfiltered data for VISLAM

	_px4_accel.update(timestamp_sample, data.accel_m_s2_x * 1000, data.accel_m_s2_y * 1000, data.accel_m_s2_z * 1000);
	_px4_gyro.update(timestamp_sample, data.gyro_rad_s_x * 1000, data.gyro_rad_s_y * 1000, data.gyro_rad_s_z * 1000);

	perf_end(_publish_perf);

	// TODO: check the return codes of this function
	return 0;
};


namespace df_mpu6050_wrapper
{

DfMPU6050Wrapper *g_dev = nullptr;

int start(enum Rotation rotation);
int stop();
int info();
void usage();

int start(enum Rotation rotation)
{
	g_dev = new DfMPU6050Wrapper(rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    IMU_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("state @ %p", g_dev);
	g_dev->info();

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_mpu6050_wrapper 'start', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_mpu6050_wrapper


int
df_mpu6050_wrapper_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			df_mpu6050_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_mpu6050_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		ret = df_mpu6050_wrapper::start(rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_mpu6050_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_mpu6050_wrapper::info();
	}

	else {
		df_mpu6050_wrapper::usage();
		return 1;
	}

	return ret;
}
