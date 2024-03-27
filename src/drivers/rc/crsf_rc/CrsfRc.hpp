/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "CrsfParser.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>

// telemetry
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>

using namespace device;

class CrsfRc : public ModuleBase<CrsfRc>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CrsfRc(const char *device);
	~CrsfRc() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;

	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};

	input_rc_s _input_rc{};

	bool SendTelemetryBattery(const uint16_t voltage, const uint16_t current, const int fuel, const uint8_t remaining);

	bool SendTelemetryGps(const int32_t latitude, const int32_t longitude, const uint16_t groundspeed,
			      const uint16_t gps_heading, const uint16_t altitude, const uint8_t num_satellites);

	bool SendTelemetryAttitude(const int16_t pitch, const int16_t roll, const int16_t yaw);

	bool SendTelemetryFlightMode(const char *flight_mode);

	Serial *_uart = nullptr; ///< UART interface to RC

	char _device[20] {}; ///< device / serial port path
	bool _is_singlewire{false};

	static constexpr size_t RC_MAX_BUFFER_SIZE{64};
	uint8_t _rcs_buf[RC_MAX_BUFFER_SIZE] {};
	uint32_t _bytes_rx{0};

	hrt_abstime _last_packet_seen{0};

	CrsfParserStatistics_t _packet_parser_statistics{0};

	// telemetry
	hrt_abstime _telemetry_update_last{0};
	static constexpr int num_data_types{4}; ///< number of different telemetry data types
	int _next_type{0};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	enum class crsf_frame_type_t : uint8_t {
		gps = 0x02,
		battery_sensor = 0x08,
		link_statistics = 0x14,
		rc_channels_packed = 0x16,
		attitude = 0x1E,
		flight_mode = 0x21,

		// Extended Header Frames, range: 0x28 to 0x96
		device_ping = 0x28,
		device_info = 0x29,
		parameter_settings_entry = 0x2B,
		parameter_read = 0x2C,
		parameter_write = 0x2D,
		command = 0x32
	};

	enum class crsf_payload_size_t : uint8_t {
		gps = 15,
		battery_sensor = 8,
		link_statistics = 10,
		rc_channels = 22, ///< 11 bits per channel * 16 channels = 22 bytes.
		attitude = 6,
	};

	void WriteFrameHeader(uint8_t *buf, int &offset, const crsf_frame_type_t type, const uint8_t payload_size);
	void WriteFrameCrc(uint8_t *buf, int &offset, const int buf_size);

	perf_counter_t	_cycle_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t	_publish_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RC_CRSF_TEL_EN>) _param_rc_crsf_tel_en
	)
};
