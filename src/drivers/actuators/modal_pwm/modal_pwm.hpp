/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <math.h>

#include <board_config.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
// #include <px4_arch/io_timer.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/modal_io_data.h>

#include "modal_io_serial.hpp"

#include "qc_esc_packet.h"
#include "qc_esc_packet_types.h"

using namespace time_literals;

class ModalPWM final : public ModuleBase<ModalPWM>, public OutputModuleInterface
{
public:
	ModalPWM();
	~ModalPWM() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;
			   
	struct Command {
		uint16_t	id                 = 0;
		uint8_t 	len                = 0;
		uint16_t	repeats            = 0;
		uint16_t	repeat_delay_us    = 2000;
		uint8_t		retries            = 0;
		bool		response           = false;
		uint16_t	resp_delay_us      = 1000;
		bool		print_feedback     = false;

		static const uint8_t BUF_SIZE = 128;
		uint8_t 	buf[BUF_SIZE];

		bool valid() const { return len > 0; }
		void clear() { len = 0; }
	};

	int send_cmd_thread_safe(Command *cmd);

private:
	void Run() override;
	bool stop_all_pwms();
	
	/* Parameters */
	static constexpr uint32_t MODAL_PWM_UART_CONFIG = 1;
	// const char * MODAL_PWM_DEFAULT_PORT = "7";
	static constexpr uint32_t MODAL_PWM_DEFAULT_BAUD = 921600;
	static constexpr uint16_t MODAL_PWM_OUTPUT_CHANNELS = 4;
	static constexpr uint16_t MODAL_PWM_OUTPUT_DISABLED = 0;

	static constexpr uint32_t MODAL_PWM_WRITE_WAIT_US = 200;
	static constexpr uint32_t MODAL_PWM_DISCONNECT_TIMEOUT_US = 500000;

	static constexpr uint16_t DISARMED_VALUE = 0;

	static constexpr uint16_t MODAL_PWM_DEFAULT_PWM_MIN = 0;
	static constexpr uint16_t MODAL_PWM_DEFAULT_PWM_MAX = 800;
	static constexpr uint16_t MODAL_PWM_DEFAULT_PWM_FAILSAFE = 0;

	static constexpr float    MODAL_PWM_MODE_DISABLED_SETPOINT = -0.1f;
	static constexpr float    MODAL_PWM_MODE_THRESHOLD = 0.0f;

	static constexpr uint32_t MODAL_PWM_MODE = 0;

	const char *_device = MODAL_PWM_DEFAULT_PORT;

	typedef struct {
		int32_t		config{MODAL_PWM_UART_CONFIG};
		int32_t		mode{MODAL_PWM_MODE};
		int32_t		baud_rate{MODAL_PWM_DEFAULT_BAUD};
		int32_t		pwm_min{MODAL_PWM_DEFAULT_PWM_MIN};
		int32_t		pwm_max{MODAL_PWM_DEFAULT_PWM_FAILSAFE};
		int32_t		pwm_failsafe{MODAL_PWM_DEFAULT_PWM_MAX};
		int32_t		function_map[MODAL_PWM_OUTPUT_CHANNELS] {0, 0, 0, 0};
		int32_t		motor_map[MODAL_PWM_OUTPUT_CHANNELS] {1, 2, 3, 4};
		int32_t		direction_map[MODAL_PWM_OUTPUT_CHANNELS] {1, 1, 1, 1};
		int32_t		verbose_logging{0};
		int32_t 	publish_battery_status{0};
	} modal_pwm_params_t;
	modal_pwm_params_t	_parameters;

	/* QUP7, VOXL2 J19, /dev/slpi-uart-7*/
	ModalIoSerial 		*_uart_port;
	
	MixingOutput 	_mixing_output;
	unsigned		_current_update_rate{0};

	// int _timer_rates[MAX_IO_TIMERS] {};

	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 	_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription 	_actuator_test_sub{ORB_ID(actuator_test)};

	// unsigned	_num_outputs{DIRECT_PWM_OUTPUT_CHANNELS};

	bool		_pwm_on{false};
	int32_t		_pwm_fullscale{0};
	int16_t 	_pwm_values[MODAL_PWM_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	bool		_first_update_cycle{true};

	typedef struct {
		uint8_t		number;
		int8_t		direction;
	} ch_assign_t;
	
	ch_assign_t		_output_map[MODAL_PWM_OUTPUT_CHANNELS] {{1, 1}, {2, 1}, {3, 1}, {4, 1}};

	perf_counter_t		_cycle_perf;
	perf_counter_t		_interval_perf;

	uint16_t		_cmd_id{0};
	Command 		_current_cmd;
	px4::atomic<Command *>	_pending_cmd{nullptr};

	static const uint8_t 	READ_BUF_SIZE = 128;
	uint8_t			_read_buf[READ_BUF_SIZE];

	int	load_params(modal_pwm_params_t *params, ch_assign_t *map);
	void update_params();
	int	flush_uart_rx();
	int read_response(Command *out_cmd);
};
