/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file lightware_grf_serial.hpp
 * @author Aaron Porter <aaron.porter@ascendengineer.com>
 *
 * Serial Protocol driver for the Lightware GRF rangefinder series
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/distance_sensor.h>

#include "grf_commands.h"

enum GRF_SERIAL_STATE {
	STATE_UNINIT = 0,
	STATE_ACK_PRODUCT_NAME = 1,
	STATE_ACK_UPDATE_RATE = 2,
	STATE_ACK_DISTANCE_OUTPUT = 3,
	STATE_SEND_STREAM = 4,
};

enum GRF_PARSED_STATE {
	START = 0,
	FLG_LOW,
	FLG_HIGH,
	ID,
	DATA,
	CRC_LOW,
	CRC_HIGH
};

enum MODEL {
	GRF250 = 0,
	GRF500 = 1
};

using namespace time_literals;
class GRFLaserSerial : public px4::ScheduledWorkItem
{
public:
	GRFLaserSerial(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~GRFLaserSerial() override;

	int				init();
	void				print_info();
	void				grf_get_and_handle_request(const int payload_length, const GRF_SERIAL_CMD msg_id);
	void				grf_send(uint8_t msg_id, bool r_w, int32_t *data, uint8_t data_len);
	uint16_t			grf_format_crc(uint16_t crc, uint8_t data_value);
	void				grf_process_replies();

private:

	distance_sensor_s		_distance{};
	static constexpr uint64_t 	GRF_MEAS_TIMEOUT{100_ms};
	static constexpr float 		GRF_SCALE_FACTOR = 0.01f;

	void				start();
	void				stop();
	void				Run() override;
	int				measure();
	int				collect();
	bool				_crc_valid{false};

	PX4Rangefinder                  _px4_rangefinder;
	char 				_port[20] {};
	int				_interval{200000};
	bool				_collect_phase{false};
	int 				_fd{-1};
	uint8_t				_linebuf[GRF_MAX_PAYLOAD] {};
	int				_linebuf_size{0};

	// GRF uses a binary protocol to include header,flags
	// message ID, payload, and checksum
	GRF_SERIAL_STATE		_sensor_state{STATE_UNINIT};
	int				_baud_rate{0};
	int32_t				_product_name[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int32_t				_stream_data{0};
	int32_t				_update_rate{0};
	int32_t				_model_type{0};
	int32_t				_data_output{0};
	const uint8_t			_start_of_frame{0xAA};
	uint16_t			_data_bytes_recv{0};
	uint8_t				_parsed_state{0};
	uint16_t			_calc_crc{0};

	// end of GRF data members

	hrt_abstime			_last_received_time{0};
	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;
};
