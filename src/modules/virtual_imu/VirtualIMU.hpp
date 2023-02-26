/****************************************************************************
 *
 *   Copyright (c) 2023-2023 PX4 Development Team. All rights reserved.
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

#ifndef VIRTUAL_IMU_HPP
#define VIRTUAL_IMU_HPP

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include "RingBuffer.hpp"

using namespace time_literals;

class VirtualIMU : public ModuleBase<VirtualIMU>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VirtualIMU();
	~VirtualIMU() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	static constexpr int MAX_SENSOR_COUNT = 3;
	static constexpr int MAX_TIMESTAMP_GYRO_DT_US  = 150;
	static constexpr int MAX_TIMESTAMP_ACCEL_DT_US = 150;

	void process_gyro();
	void process_accel();

	float median(float x, float y, float z);
	size_t find_median_index(float x, float y, float z);

	void check_newest_timestamp_and_register_callback(sensor_gyro_fifo_s *sensor_gyro_fifo,
			sensor_accel_fifo_s *sensor_accel_fifo);

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	uint32_t _accel_device_id{0};
	uint32_t _gyro_device_id{0};

	uint64_t _last_gyro_timestamp{0};
	uint64_t _last_accel_timestamp{0};

	uint16_t _median_gyro_dt_us{MAX_TIMESTAMP_GYRO_DT_US};
	uint16_t _median_accel_dt_us{MAX_TIMESTAMP_ACCEL_DT_US};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionMultiArray<sensor_gyro_fifo_s, MAX_SENSOR_COUNT> _gyro_fifo_subs{ORB_ID::sensor_gyro_fifo};
	uORB::SubscriptionMultiArray<sensor_accel_fifo_s, MAX_SENSOR_COUNT> _accel_fifo_subs{ORB_ID::sensor_accel_fifo};

	uORB::SubscriptionCallbackWorkItem _sensor_gyro_fifo_sub_callback{this, ORB_ID(sensor_gyro_fifo)};
	uORB::SubscriptionCallbackWorkItem _sensor_accel_fifo_sub_callback{this, ORB_ID(sensor_accel_fifo)};

	uORB::SubscriptionMultiArray<sensor_gyro_s, MAX_SENSOR_COUNT> _sensor_gyro_subs{ORB_ID::sensor_gyro};
	uORB::SubscriptionMultiArray<sensor_accel_s, MAX_SENSOR_COUNT> _sensor_accel_subs{ORB_ID::sensor_accel};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _cycle_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _fft_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": virtual_imu")};
	perf_counter_t _gyro_generation_gap_perf{nullptr};
	perf_counter_t _gyro_fifo_generation_gap_perf{nullptr};

	unsigned _gyro_last_generation[MAX_SENSOR_COUNT] {0};
	unsigned _accel_last_generation[MAX_SENSOR_COUNT] {0};

	struct gyroFIFOSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		int16_t data[3] {};
		float dt{0.f};
		float scale{0.f};
	};

	struct accelFIFOSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		int16_t data[3] {};
		float dt{0.f};
		float scale{0.f};
	};

	RingBuffer<gyroFIFOSample, 32 * sensor_gyro_fifo_s::ORB_QUEUE_LENGTH> _gyro_fifo_buffer[MAX_SENSOR_COUNT] {};
	RingBuffer<accelFIFOSample, 32 * 2> _accel_fifo_buffer[MAX_SENSOR_COUNT] {};

	enum class STATE : uint8_t {
		CONFIGURE,
		RUN,
	} _state{STATE::CONFIGURE};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VIRTUAL_IMU_EN>) _param_virtual_imu_en,
		(ParamInt<px4::params::GYRO_AXIS_MEDIAN>) _param_gyro_axis_median
	)
};

#endif // !VIRTUAL_IMU_HPP
