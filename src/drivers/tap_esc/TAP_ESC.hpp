/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include <errno.h>

#include <math.h>	// NAN
#include <cstring>

#include <lib/drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/cdev/CDev.hpp>
#include <lib/led/led.h>
#include <lib/tunes/tunes.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/led_control.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include "tap_esc_common.h"

#include "drv_tap_esc.h"

#if !defined(BOARD_TAP_ESC_MODE)
#  define BOARD_TAP_ESC_MODE 0
#endif

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

using namespace time_literals;

/*
 * This driver connects to TAP ESCs via serial.
 */
class TAP_ESC : public cdev::CDev, public ModuleBase<TAP_ESC>, public OutputModuleInterface
{
public:
	TAP_ESC(const char *device, uint8_t channels_count);
	virtual ~TAP_ESC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int init() override;
	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	void Run() override;

	inline void send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt);
	inline void send_tune_packet(EscbusTunePacket &tune_packet);

	MixingOutput _mixing_output;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _initialized{false};
	char _device[DEVICE_ARGUMENT_MAX_LENGTH] {};
	int _uart_fd{-1};

	const uint8_t _device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;
	const uint8_t _device_dir_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_DIR;

	uORB::PublicationMulti<esc_status_s> _esc_feedback_pub{ORB_ID(esc_status)};
	esc_status_s      _esc_feedback{};
	uint8_t    	  _channels_count{0}; 		///< number of ESC channels
	uint8_t 	  _responding_esc{0};

	ESC_UART_BUF _uartbuf{};
	EscPacket    _packet{};

	Tunes _tunes{};
	uORB::Subscription _tune_control_sub{ORB_ID(tune_control)};
	hrt_abstime _interval_timestamp{0};
	unsigned int _silence_length{0};	///< If nonzero, silence before next note.
	unsigned int _frequency{0};
	unsigned int _duration{0};

	LedControlData _led_control_data{};
	LedController  _led_controller{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
