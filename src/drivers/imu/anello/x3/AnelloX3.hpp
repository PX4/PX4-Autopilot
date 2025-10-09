/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file anello_x3.cpp
 *
 * Driver for the Anello X3 IMU
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <uORB/topics/distance_sensor.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>


#define APCAL    255
#define APMCA 	 254
#define APIMU 	 253
#define APSIPHOG 252
#define APMAG    251
#define APCFG    249
#define APRST    248
#define APPNG    247
#define APECH    246
#define APVER    245
#define APSER    244
#define APPID    243
#define APUNL    242
#define APFLA    241
#define APSEN    240

using namespace time_literals;

class AnelloX3 : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	AnelloX3(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~AnelloX3();

	int init();

	void print_info();

	void calculate_checksum(char *buf_start, char* buf_end, char *checksum_str);

	void start_config_mode();
	void stop_config_mode();
	int get_config_mode();

	void send_x3_request(const char* msg, size_t len);
	int get_x3_response(char *resposne);

private:

	int collect();

	void Run() override;

	int ConfigureUART();
	int ConfigureX3();
	void start();
	void stop();


	void ParseInput();
	void ParseBinary();
	void ParseAscii();

	PX4Accelerometer _px4_accel_og;
	PX4Gyroscope _px4_gyro_og;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	PX4Magnetometer _px4_mag;

	char _linebuf[512] {};
	unsigned int _linebuf_index{0};



	unsigned _bytes_read{0};

	static constexpr int kCONVERSIONINTERVAL{9_ms};

	//int _fd{-1}; //original code.

	px4::atomic_bool _print_debug{false};

	hrt_abstime _last_read{0};

	float _mag_prev[3] {};

	int _fd{-1};
	char _port[20] {};

	int _config_mode = 0;
	int _send_req_flag = 0;
	int _resp_received_flag = 1;
	char _x3_request[256] {};
	char _x3_response[256] {};

	void generate_x3_request(const char* msg, char* full_msg, size_t len);
	ssize_t write_x3();

	uint8_t is_message_type_valid(const char *message_type);

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _poll_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": poll timeout")};
	perf_counter_t _poll_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": poll error")};
	perf_counter_t _read_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": read error")};

	perf_counter_t _checksum_good_perf{perf_alloc(PC_COUNT, MODULE_NAME": checksum good")};
	perf_counter_t _checksum_bad_perf{perf_alloc(PC_COUNT, MODULE_NAME": checksum bad")};

	perf_counter_t _publish_accel_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish accel interval")};
	perf_counter_t _publish_gyro_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": publish gyro interval")};

	typedef union __attribute__((__packed__, aligned(1)))
	{
		uint8_t bytes[61]; // total size of the message

		struct __attribute__((__packed__, aligned(1)))
		{
			uint8_t preamble[2]; // index 0 and 1
			uint8_t msg_type; // index 2
			uint8_t payload_length; // index 3
			uint64_t mcu_time; // index 4 through 11
			uint64_t sync_time; // index 12 through 19
			int16_t ax1; // index 20 through 21
			int16_t ay1; // index 22 through 23
			int16_t az1; // index 24 through 25
			int16_t wx1; // index 26 through 27
			int16_t wy1; // index 28 through 29
			int16_t wz1; // index 30 through 31
			int32_t og_wx; // index 32 through 35
			int32_t og_wy; // index 36 through 39
			int32_t og_wz; // index 40 through 43
			int16_t mag_x; // index 44 through 45
			int16_t mag_y; // index 46 through 47
			int16_t mag_z; // index 48 through 49
			int16_t temperature; // index 50 through 51
			uint16_t mems_range; // index 52 through 53
			uint16_t fog_range; // index 54 through 55
			uint8_t status_x; // index 56
			uint8_t status_y; // index 57
			uint8_t status_z; // index 58
			uint8_t checksum[2]; // index 59 and 60
		} fields;
	} imu_msg_t;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::X3_ODATA_RATE>) _x3_odr,
		(ParamInt<px4::params::X3_LPA>) _x3_lpa,
		(ParamInt<px4::params::X3_LPW>) _x3_lpw,
		(ParamInt<px4::params::X3_LPO>) _x3_lpo
	);
};


