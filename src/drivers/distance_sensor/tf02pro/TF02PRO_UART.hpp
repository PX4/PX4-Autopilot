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
 * @file TF02PRO_UART.hpp
 *
 * UART driver for the Benewake TF02 Pro distance sensor.
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#include "tf02pro_parser.h"

/* Physical constants — identical to the I2C driver */
#define TF02PRO_UART_MIN_DISTANCE  (0.10f)
#define TF02PRO_UART_MAX_DISTANCE  (35.00f)
#define TF02PRO_DEFAULT_PORT       "/dev/ttyS3"

using namespace time_literals;

class TF02PRO_UART : public px4::ScheduledWorkItem
{
public:
	TF02PRO_UART(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~TF02PRO_UART() override;

	int  init();
	void print_info();

private:
	int  collect();
	void Run() override;
	void start();
	void stop();

	PX4Rangefinder _px4_rangefinder;

	TF02PRO_PARSE_STATE _parse_state{TF02PRO_PARSE_STATE::STATE0_UNSYNC};
	uint8_t  _uart_buf[9]{};
	unsigned _uart_buf_idx{0};

	char _port[20]{};

	// Sensor configured at 250 Hz; poll slightly faster to catch every frame
	static constexpr int kCONVERSIONINTERVAL{4_ms};

	int         _fd{-1};
	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT,   MODULE_NAME": uart_com_err")};
	perf_counter_t _sample_perf {perf_alloc(PC_ELAPSED, MODULE_NAME": uart_read")};
};
