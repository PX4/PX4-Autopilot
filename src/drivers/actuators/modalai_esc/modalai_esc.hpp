/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <drivers/device/device.h>
#include <drivers/drv_mixer.h>
#include <lib/cdev/CDev.hpp>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/esc_status.h>

#include "modalai_esc_serial.hpp"

#include "qc_esc_packet.h"
#include "qc_esc_packet_types.h"

class ModalaiEsc : public cdev::CDev, public ModuleBase<ModalaiEsc>, public OutputModuleInterface
{
public:
	ModalaiEsc();
	virtual ~ModalaiEsc();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	/** @see OutputModuleInterface */
	void mixerChanged() override;

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	typedef enum {
		UART_ESC_RESET,
		UART_ESC_VERSION,
		UART_ESC_TONE,
		UART_ESC_LED
	} uart_esc_cmd_t;

	struct Command {
		uint16_t	id                 = 0;
		uint8_t 	len                = 0;
		uint16_t	repeats            = 0;
		uint16_t	repeat_delay_us    = 2000;
		uint8_t		retries            = 0;
		bool		  response           = false;
		uint16_t	resp_delay_us      = 1000;
		bool      print_feedback     = false;

		static const uint8_t BUF_SIZE = 128;
		uint8_t 	buf[BUF_SIZE];

		bool valid() const { return len > 0; }
		void clear() { len = 0; }
	};

	int sendCommandThreadSafe(Command *cmd);

private:
	static constexpr uint32_t MODALAI_ESC_UART_CONFIG = 1;
	static constexpr uint32_t MODALAI_ESC_DEFAULT_BAUD = 250000;
	static constexpr uint16_t MODALAI_ESC_OUTPUT_CHANNELS = 4;
	static constexpr uint16_t MODALAI_ESC_OUTPUT_DISABLED = 0;

	static constexpr uint32_t MODALAI_ESC_WRITE_WAIT_US = 200;
	static constexpr uint32_t MODALAI_ESC_DISCONNECT_TIMEOUT_US = 500000;

	static constexpr uint16_t DISARMED_VALUE = 0;

	static constexpr uint16_t MODALAI_ESC_PWM_MIN = 0;
	static constexpr uint16_t MODALAI_ESC_PWM_MAX = 800;
	static constexpr uint16_t MODALAI_ESC_DEFAULT_RPM_MIN = 5000;
	static constexpr uint16_t MODALAI_ESC_DEFAULT_RPM_MAX = 17000;

	static constexpr float    MODALAI_ESC_MODE_DISABLED_SETPOINT = -0.1f;
	static constexpr float    MODALAI_ESC_MODE_THRESHOLD = 0.0f;

	static constexpr float    MODALAI_ESC_MODE_DEAD_ZONE_MIN = 0.0f;
	static constexpr float    MODALAI_ESC_MODE_DEAD_ZONE_MAX = 1.0f;
	static constexpr float    MODALAI_ESC_MODE_DEAD_ZONE_1 = 0.30f;
	static constexpr float    MODALAI_ESC_MODE_DEAD_ZONE_2 = 0.02f;

	static constexpr uint32_t MODALAI_ESC_MODE = 0;
	static constexpr uint32_t MODALAI_ESC_MODE_TURTLE_AUX1 = 1;
	static constexpr uint32_t MODALAI_ESC_MODE_TURTLE_AUX2 = 2;
	static constexpr uint32_t MODALAI_ESC_MODE_UART_BRIDGE = 3;

	//static constexpr uint16_t max_pwm(uint16_t pwm) { return math::min(pwm, MODALAI_ESC_PWM_MAX); }
	//static constexpr uint16_t max_rpm(uint16_t rpm) { return math::min(rpm, MODALAI_ESC_RPM_MAX); }

	ModalaiEscSerial 	*_uart_port;
	ModalaiEscSerial 	*_uart_port_bridge;

	typedef struct {
		int32_t		config{MODALAI_ESC_UART_CONFIG};
		int32_t		mode{MODALAI_ESC_MODE};
		float		dead_zone_1{MODALAI_ESC_MODE_DEAD_ZONE_1};
		float		dead_zone_2{MODALAI_ESC_MODE_DEAD_ZONE_2};
		int32_t		baud_rate{MODALAI_ESC_DEFAULT_BAUD};
		int32_t		rpm_min{MODALAI_ESC_DEFAULT_RPM_MIN};
		int32_t		rpm_max{MODALAI_ESC_DEFAULT_RPM_MAX};
		int32_t		motor_map[MODALAI_ESC_OUTPUT_CHANNELS] {1, 2, 3, 4};
	} uart_esc_params_t;

	struct EscChan {
		int16_t	  rate_req;
		uint8_t		state;
		uint16_t	rate_meas;
		uint8_t   power_applied;
		uint8_t		led;
		uint8_t		cmd_counter;
		float 		voltage;  //Volts
		float     current;  //Amps
		float     temperature; //deg C
		hrt_abstime feedback_time;
	};

	typedef struct {
		uint8_t		number;
		int8_t		direction;
	} ch_assign_t;

	typedef struct {
		led_control_s           control{};
		vehicle_control_mode_s  mode{};
		uint8_t                 led_mask;// TODO led_mask[MODALAI_ESC_OUTPUT_CHANNELS];
		bool                    breath_en;
		uint8_t	                breath_counter;
		bool                    test;
	} led_rsc_t;

	ch_assign_t		_output_map[MODALAI_ESC_OUTPUT_CHANNELS] {{1, 1}, {2, 1}, {3, 1}, {4, 1}};
	MixingOutput 		_mixing_output{"MODALAI_ESC", MODALAI_ESC_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	int			_class_instance{-1};

	perf_counter_t		_cycle_perf;
	perf_counter_t		_output_update_perf;

	bool			_outputs_on{false};

	unsigned		_current_update_rate{0};

	uORB::Subscription	_vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription  _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription 	_parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription	_led_update_sub{ORB_ID(led_control)};

	//uORB::Publication<actuator_outputs_s> _outputs_debug_pub{ORB_ID(actuator_outputs_debug)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	uart_esc_params_t	_parameters;
	int			update_params();
	int			load_params(uart_esc_params_t *params, ch_assign_t *map);

	bool			_turtle_mode_en{false};
	int32_t			_rpm_fullscale{0};
	manual_control_setpoint_s _manual_control_setpoint{};

	uint16_t		_cmd_id{0};
	Command 		_current_cmd;
	px4::atomic<Command *>	_pending_cmd{nullptr};

	EscChan			_esc_chans[MODALAI_ESC_OUTPUT_CHANNELS] {};
	Command			_esc_cmd;
	esc_status_s _esc_status;
	EscPacket   _fb_packet;
	EscPacket   _uart_bridge_packet;

	led_rsc_t	 	_led_rsc;
	int         _fb_idx;

	static const uint8_t READ_BUF_SIZE = 128;
	uint8_t     _read_buf[READ_BUF_SIZE];

	void 			updateLeds(vehicle_control_mode_s mode, led_control_s control);

	int			populateCommand(uart_esc_cmd_t cmd_type, uint8_t cmd_mask, Command *out_cmd);
	int 			readResponse(Command *out_cmd);
	int 			parseResponse(uint8_t *buf, uint8_t len, bool print_feedback);
	int			flushUartRx();
	int			checkForEscTimeout();
};
