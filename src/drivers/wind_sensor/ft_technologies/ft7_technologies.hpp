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
 * @file ft7_technolofies.hpp
 * @author Henry Kotze <henry@flycloudline.com>
 *
 * Driver for the FT Technology Wind Sensor. FT742
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_airflow.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <drivers/drv_hrt.h>

using namespace time_literals;


class Ft7Technologies : public px4::ScheduledWorkItem
{
public:
	Ft7Technologies(const char *port);
	~Ft7Technologies() override;

	int 			init();
	void				print_info();

private:

	void				start();
	void				stop();
	void				Run() override;
	int				measure();
	int				collect();
	bool 				checksum(char *buf, uint32_t checksum);
	uint8_t				hex2int(char ch);

	uORB::Publication<sensor_airflow_s> _sensor_airflow_pub{ORB_ID(sensor_airflow)};

	char 				_port[20] {};
	char 				_readbuf[30] {};
	uint64_t         		_interval{100000};
	bool				_collect_phase{false};
	int				_fd{-1};
	char				_linebuf[20] {};
	hrt_abstime			_last_read{0};
	hrt_abstime			_last_measure{0};

	char 				_raw_angle[5];
	char 				_raw_speed[5];
	char 				_raw_status[2];
	char 				_raw_checksum[3];

	uint16_t 			_checksum{0};
	uint32_t 			_hex_checksum{0};
	int				_msg_part_counter{0};
	int				_byte_counter{0};
	int				_msg_byte_counter{0};
	uint32_t			_checksum_counter{0};

	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

};
