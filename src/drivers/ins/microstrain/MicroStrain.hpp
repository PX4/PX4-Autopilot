/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_baro.h>

#include "mip_sdk/src/mip/mip_all.h"

#include "modal_io_serial.hpp"

using namespace mip::C;

using namespace time_literals;

class MicroStrain : public ModuleBase<MicroStrain>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MicroStrain(const char *_device);
	~MicroStrain() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	/* Callbacks */

	// Sensor Callbacks
	static void sensorCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp);


private:
	/** @see ModuleBase */
	void Run() override;

	/// @brief Attempt to connect to the Sensor and set in known configuration
	bool initializeIns();

	mip_cmd_result getSupportedDescriptors();

	bool supportsDescriptor(uint8_t descriptor_set, uint8_t field_descriptor);

	bool supportsDescriptorSet(uint8_t descriptor_set);

	mip_cmd_result getBaseRate(uint8_t descriptor_set, uint16_t *base_rate);

	mip_cmd_result configureImuMessageFormat();

	mip_cmd_result writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors,
					  const mip::DescriptorRate *descriptors);

	int connectAtBaud(int32_t baud);

	mip::CmdResult forceIdle();

	mip_cmd_result writeBaudRate(uint32_t baudrate, uint8_t port);

	bool init();

	uint32_t _dev_id{0};

	// Sensor types needed for message creation / updating / publishing
	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope _px4_gyro{0};
	PX4Magnetometer _px4_mag{0};
	sensor_baro_s _sensor_baro{0};

	// Must publish to prevent sensor stale failure (sensors module)
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	uint8_t _parse_buffer[2048];
	bool _is_initialized{false};
	int _ms_schedule_rate_us{0};

	uint16_t _supported_descriptors[1024] = {0};
	uint16_t _supported_desc_len = 0;
	uint16_t _supported_descriptor_sets[1024] = {0};
	uint16_t _supported_desc_set_len = 0;

	mip::C::mip_interface _device;

	// Handlers
	mip_dispatch_handler _sensor_data_handler;

	char _port[128];

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MS_IMU_RATE_HZ>) _param_ms_imu_rate_hz,
		(ParamInt<px4::params::MS_MAG_RATE_HZ>) _param_ms_mag_rate_hz,
		(ParamInt<px4::params::MS_BARO_RATE_HZ>) _param_ms_baro_rate_hz
	)

};
