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
#include <drivers/drv_hrt.h>
#include <lib/rc/sbus.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>

#include "voxl2_io_serial.hpp"

#include "voxl2_io_packet.h"
#include "voxl2_io_packet_types.h"

using namespace time_literals;

class Voxl2IO final : public ModuleBase<Voxl2IO>, public OutputModuleInterface
{
public:
	Voxl2IO();
	~Voxl2IO() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[input_rc_s::RC_INPUT_MAX_CHANNELS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	virtual int	init();

	void update_pwm_config();
	int get_version_info();

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
	int receive_sbus();

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi, input_rc_s &input_rc);
private:
	void Run() override;
	bool stop_all_pwms();

	/* PWM Parameters */
	static constexpr uint32_t VOXL2_IO_CONFIG = 0;				// Default to off
	static constexpr uint32_t VOXL2_IO_BOARD_CONFIG_SIZE = 4;	// PWM_MIN, PWM_MAX, 4 bytes
	static constexpr uint32_t VOXL2_IO_ESC_CAL_SIZE = 1;
	static constexpr uint32_t VOXL2_IO_DEFAULT_BAUD = 921600;
	static constexpr uint16_t VOXL2_IO_OUTPUT_CHANNELS = 4;
	static constexpr uint16_t VOXL2_IO_OUTPUT_DISABLED = 0;

	static constexpr uint32_t VOXL2_IO_WRITE_WAIT_US = 200;
	static constexpr uint32_t VOXL2_IO_DISCONNECT_TIMEOUT_US = 500000;

	static constexpr uint16_t DISARMED_VALUE = 0;

	static constexpr uint16_t VOXL2_IO_MIXER_MIN = 0;
	static constexpr uint16_t VOXL2_IO_MIXER_MAX = 800;
	static constexpr uint16_t VOXL2_IO_MIXER_FAILSAFE = 0;
	static constexpr uint16_t VOXL2_IO_MIXER_DISARMED = 0;

	static constexpr int32_t VOXL2_IO_DEFAULT_MIN = 1000;
	static constexpr int32_t VOXL2_IO_DEFAULT_MAX = 2000;
	static constexpr int32_t VOXL2_IO_DEFAULT_FAILSAFE = 900;
	static constexpr int32_t VOXL2_IO_TICS = 24;	// 24 tics per us on M0065 timer clks

	/* SBUS */
	static constexpr uint16_t VOXL2_IO_SBUS_FRAME_SIZE = 30;
	static constexpr uint16_t SBUS_PAYLOAD = 3;

	/* M0065 version info */
	static constexpr uint16_t VOXL2_IO_VERSION_INFO_SIZE = 6;
	static constexpr uint16_t VOXL2_IO_SW_PROTOCOL_VERSION = 1;
	static constexpr uint16_t VOXL2_IO_HW_PROTOCOL_VERSION = 35;
	VOXL2_IO_VERSION_INFO _version_info;

	/* Module update interval */
	static constexpr unsigned	_current_update_interval{4000}; // 250 Hz

	typedef struct {
		int32_t		config{VOXL2_IO_CONFIG};
		int32_t		baud_rate{VOXL2_IO_DEFAULT_BAUD};
		int32_t		pwm_min{VOXL2_IO_DEFAULT_MIN};
		int32_t		pwm_max{VOXL2_IO_DEFAULT_MAX};
		int32_t		pwm_failsafe{VOXL2_IO_DEFAULT_FAILSAFE};
		int32_t 	param_rc_input_proto{0};
		int32_t		param_rc_rssi_pwm_chan{0};
		int32_t		function_map[VOXL2_IO_OUTPUT_CHANNELS] {0, 0, 0, 0};
		int32_t		verbose_logging{0};
	} voxl2_io_params_t;
	voxl2_io_params_t	_parameters;

	typedef enum {
		PWM_MODE_START = 0,
		PWM_MODE_400,
		PWM_MODE_END
	} PWM_MODE;

	enum RC_MODE {
		DISABLED = 0,
		SBUS,
		SPEKTRUM,
		EXTERNAL,
		SCAN
	} _rc_mode{RC_MODE::SCAN};

	/* QUP7, VOXL2 J19, /dev/slpi-uart-7*/
	char 				_device[10] {VOXL2_IO_DEFAULT_PORT};
	Voxl2IoSerial 		*_uart_port;

	/* Mixer output */
	MixingOutput 	_mixing_output;

	/* RC input */
	VOXL2_IOPacket _sbus_packet;
	uint64_t _rc_last_valid;
	uint16_t _raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {UINT16_MAX};
	unsigned _sbus_frame_drops{0};
	uint16_t _sbus_total_frames{0};
	bool	 _new_packet{false};

	/* Publications */
	uORB::PublicationMulti<input_rc_s> _rc_pub{ORB_ID(input_rc)};

	/* Subscriptions */
	uORB::Subscription 	_parameter_update_sub{ORB_ID(parameter_update)};

	bool		_pwm_on{false};
	int32_t		_pwm_fullscale{0};
	int16_t 	_pwm_values[VOXL2_IO_OUTPUT_CHANNELS] = {0, 0, 0, 0};
	bool		_outputs_disabled{false};

	perf_counter_t		_cycle_perf;
	perf_counter_t		_output_update_perf;

	bool 			_debug{false};
	uint16_t		_cmd_id{0};
	Command 		_current_cmd;
	px4::atomic<Command *>	_pending_cmd{nullptr};

	static const uint8_t 	READ_BUF_SIZE = 128;
	uint8_t			_read_buf[READ_BUF_SIZE];
	uint32_t		_bytes_sent{0};
	uint32_t		_bytes_received{0};
	uint32_t		_packets_sent{0};
	uint32_t		_packets_received{0};

	int parse_response(uint8_t *buf, uint8_t len);
	int	load_params(voxl2_io_params_t *params);
	int update_params();
	int	flush_uart_rx();
	int calibrate_escs();
};
