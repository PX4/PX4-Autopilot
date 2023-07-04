/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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
 * @file lightware_sf45_serial.hpp
 * @author Andrew Brahim <dirksavage88@gmail.com>
 *
 * Serial Protocol driver for the Lightware SF45/B rangefinder series
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
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/distance_sensor.h>

#include "sf45_commands.h"
class SF45LaserSerial : public px4::ScheduledWorkItem
{
public:
	SF45LaserSerial(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~SF45LaserSerial() override;

	int 			        init();
	void				print_info();
	void                            sf45_request_handle(int val, uint8_t *value);
	void                            sf45_send(uint8_t msg_id, bool r_w, int *data, uint8_t data_len);
	uint16_t                        sf45_format_crc(uint16_t crc, uint8_t data_value);
	void                            sf45_process_replies(float *data);
	uint8_t                         sf45_convert_angle(const int16_t yaw);
	float                           sf45_wrap_360(float f);
protected:
	obstacle_distance_s                       _obstacle_map_msg{};
	uORB::Publication<obstacle_distance_s>		_obstacle_distance_pub{ORB_ID(obstacle_distance)};	/**< obstacle_distance publication */

private:

	void				start();
	void				stop();
	void				Run() override;
	int				measure();
	int				collect();
	bool                            _crc_valid{false};
	PX4Rangefinder                  _px4_rangefinder;

	char 				_port[20] {};
	int	        _interval{10000};
	bool				_collect_phase{false};
	int 				_fd{-1};
	int         _linebuf[256] {};
	unsigned		_linebuf_index{0};
	hrt_abstime _last_read{0};

	// SF45/B uses a binary protocol to include header,flags
	// message ID, payload, and checksum
	bool                            _is_sf45{false};
	bool                            _init_complete{false};
	bool                            _sensor_ready{false};
	uint8_t                         _sensor_state{0};
	int                             _baud_rate{0};
	int                             _product_name[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	int                             _stream_data{0};
	int32_t                         _update_rate{1};
	int                             _data_output{0};
	const uint8_t                   _start_of_frame{0xAA};
	uint16_t                        _data_bytes_recv{0};
	uint8_t                         _parsed_state{0};
	bool                            _sop_valid{false};
	uint16_t                        _calc_crc{0};
	uint8_t                         _num_retries{2};
	int32_t                         _yaw_cfg{0};
	int32_t                         _orient_cfg{0};
	int32_t                         _collision_constraint{0};
	uint16_t                        _previous_bin{0};

	// end of SF45/B data members

	unsigned			                  _consecutive_fail_count;

	perf_counter_t			            _sample_perf;
	perf_counter_t			            _comms_errors;

};
