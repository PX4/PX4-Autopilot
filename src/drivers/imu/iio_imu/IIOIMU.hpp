/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#include "IIOScan.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include <limits.h>

class IIOIMU final : public ModuleBase
{
public:
	static Descriptor desc;

	IIOIMU(const char *device_path, const char *name, uint32_t device_id, enum Rotation rotation, uint8_t samples_per_publish);
	~IIOIMU() override;

	static int task_spawn(int argc, char *argv[]);
	static IIOIMU *instantiate(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;
	void run() override;

private:
	enum class ReadResult { Ok, Missing, Error };
	static constexpr size_t MAX_READ_SCANS{64};
	static constexpr int POLL_TIMEOUT_MS{10};

	bool find_iio_device();
	bool configure_iio();
	bool configure_channels();
	bool configure_temperature();
	void set_temperature(float temperature);
	bool read_sensor_scale(size_t first_axis, iio_imu::SensorScale &sensor);
	bool read_sample_rate();
	bool check_zero_offset(const char *attribute) const;
	void disable_iio();
	void read_available();
	bool process_scan(const uint8_t *scan);
	void publish_pending();
	void reset_pending();
	void update_error_count();

	ReadResult read_attribute(const char *attribute, char *value, size_t value_len, bool optional = false) const;
	bool write_attribute(const char *attribute, const char *value) const;
	static bool is_device_name(const char *name);
	static uint32_t make_device_id(uint8_t slot);
	static bool write_text_file(const char *path, const char *value);
	static ReadResult read_text_file(const char *path, char *value, size_t value_len, bool optional = false);

	char _device_path[PATH_MAX] {};
	char _sysfs_path[PATH_MAX] {};
	char _name[128] {};
	int _fd{-1};
	bool _buffer_enabled{false};
	const uint8_t _samples_per_publish;
	float _sample_rate_hz{0.f};
	float _sample_interval_us{0.f};
	size_t _scan_size{0};
	iio_imu::Channel _channels[iio_imu::MAX_CHANNEL_COUNT] {};
	iio_imu::SensorScale _accel_scale{};
	iio_imu::SensorScale _gyro_scale{};
	iio_imu::TemperatureScale _temperature_scale{};
	bool _temperature_enabled{false};
	float _temperature{NAN};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
	sensor_accel_fifo_s _accel_fifo{};
	sensor_gyro_fifo_s _gyro_fifo{};
	uint64_t _last_timestamp_us{0};
	uint8_t _pending_samples{0};
	uint8_t _read_buffer[iio_imu::MAX_SCAN_SIZE * MAX_READ_SCANS] {};

	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": publish interval")};
	perf_counter_t _bad_read_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad read")};
	perf_counter_t _bad_scan_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad scan")};
	perf_counter_t _timestamp_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME ": timestamp gap")};
	perf_counter_t _stale_sample_perf{perf_alloc(PC_COUNT, MODULE_NAME ": stale sample")};
	perf_counter_t _bad_temperature_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad temperature")};
};
