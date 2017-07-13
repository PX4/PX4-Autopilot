#pragma once

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/conversion/rotation.h>

#include <uORB/topics/parameter_update.h>

#include <mpu9250/MPU9250.hpp>
#include <DevMgr.hpp>

using namespace DriverFramework;

class DfMpu9250Wrapper : public DriverFramework::MPU9250
{
public:
	DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation);
	~DfMpu9250Wrapper();

	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int stop();

	/**
	 * Print some debug info.
	 */
	void info();

private:
	virtual int _publish(const struct accel_data &data) override;
	virtual int _publish(const struct gyro_data &data) override;
	virtual int _publish(const struct mag_data &data) override;

	void _update_accel_calibration();
	void _update_gyro_calibration();
	void _update_mag_calibration();

	orb_advert_t _accel_topic;
	orb_advert_t _gyro_topic;
	orb_advert_t _mag_topic;

	orb_advert_t _mavlink_log_pub;

	int _param_update_sub;

	struct accel_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _accel_calibration;

	struct gyro_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _gyro_calibration;

	struct mag_calibration_s {
		float x_offset;
		float x_scale;
		float y_offset;
		float y_scale;
		float z_offset;
		float z_scale;
	} _mag_calibration;

	int _accel_orb_class_instance;
	int _gyro_orb_class_instance;
	int _mag_orb_class_instance;

	Integrator _accel_int;
	Integrator _gyro_int;

	math::LowPassFilter2p _accel_filter_x;
	math::LowPassFilter2p _accel_filter_y;
	math::LowPassFilter2p _accel_filter_z;
	math::LowPassFilter2p _gyro_filter_x;
	math::LowPassFilter2p _gyro_filter_y;
	math::LowPassFilter2p _gyro_filter_z;

	perf_counter_t _accel_published;
	perf_counter_t _accel_callbacks;
	perf_counter_t _accel_interval;
	perf_counter_t _accel_range_hits;

	perf_counter_t _gyro_published;
	perf_counter_t _gyro_callbacks;
	perf_counter_t _gyro_interval;
	perf_counter_t _gyro_range_hits;

	perf_counter_t _mag_published;
	perf_counter_t _mag_callbacks;
	perf_counter_t _mag_interval;
	perf_counter_t _mag_overflows;
	perf_counter_t _mag_overruns;

	perf_counter_t _fifo_overflows;
	perf_counter_t _fifo_reads;
	perf_counter_t _fifo_avg_packets;
	perf_counter_t _fifo_corruptions;

#if MPU9250_CHECK_DUPLICATES
	perf_counter_t _accel_duplicates;
	perf_counter_t _gyro_duplicates;
	perf_counter_t _mag_duplicates;
#endif

	perf_counter_t _errors;

	hrt_abstime _last_accel_range_hit_time;
	uint64_t _last_accel_range_hit_count;

	hrt_abstime _last_gyro_range_hit_time;
	uint64_t _last_gyro_range_hit_count;

	bool _mag_enabled;

	enum Rotation _rotation;
};
