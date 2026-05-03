/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file cubered_bridge_secondary.h
 *
 * CubeRed IO Module - PX4IO simulation for CubeRed Secondary
 *
 * This module simulates PX4IO functionality on the CubeRed Secondary
 * by providing low-latency serial communication on ttyS4.
 *
 * @author PX4 Development Team
 */

#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/Serial.hpp>

#include <px4_platform_common/posix.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/perf/perf_counter.h>
#include <px4_arch/io_timer.h>
#include <px4_platform/gpio.h>

// Include PX4IO protocol definitions
#include <modules/px4iofirmware/protocol.h>

using namespace time_literals;

class CuberedBridgeSecondary : public ModuleBase
{
public:
	static Descriptor desc;

	CuberedBridgeSecondary();
	~CuberedBridgeSecondary() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int run_trampoline(int argc, char *argv[]);

	static CuberedBridgeSecondary *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();
	void run() override;

private:
	static constexpr const char *DEVICE_NAME = "/dev/ttyS1";
	static constexpr uint32_t BAUDRATE = 1500000;
	static constexpr uint32_t POLL_TIMEOUT_MS = 1; // 1ms timeout for low latency

	device::Serial _uart{};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": loop interval")};

	// PWM output state
	bool _pwm_initialized{false};
	bool _pwm_armed{false};
	uint32_t _pwm_mask{0};
	int _timer_rates[MAX_IO_TIMERS] {};

	void poll_and_process();
	void process_received_data(IOPacket &packet);
	void send_response(IOPacket &packet);
	void send_packet(IOPacket &packet);
	void send_error_response();
	void send_corrupt_response();
	void handle_read_request(IOPacket &packet);
	void handle_write_request(IOPacket &packet);
	bool validate_crc(IOPacket &packet);
	uint8_t calculate_crc(IOPacket &packet);

	int init_serial();
	int init_pwm();
	void update_pwm_outputs();
	void set_pwm_armed(bool armed);
};
