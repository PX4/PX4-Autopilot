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
#include <string>

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
#include <px4_platform_common/Serial.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>

#include "voxl2_io_packet.h"
#include "voxl2_io_packet_types.h"

using namespace device;

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

	int get_version_info();
	void print_params();

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

	int receive_uart_packets();
	int parse_sbus_packet(uint8_t *raw_data, uint32_t data_len);

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi, input_rc_s &input_rc);
private:
	void Run() override;

	/* PWM Parameters */
	static constexpr uint32_t VOXL2_IO_DEFAULT_BAUD    = 921600;
	static constexpr uint16_t VOXL2_IO_OUTPUT_CHANNELS = 8;
	static constexpr uint16_t VOXL2_IO_MIXER_FAILSAFE  =
		UINT16_MAX; //this will tell mixer to use the default failsafe depending on the function

	/* SBUS */
	static constexpr uint16_t VOXL2_IO_SBUS_FRAME_SIZE = 30;
	static constexpr uint16_t SBUS_PAYLOAD = 3;

	//packet packing function accepts pulse width in nanoseconds
	//assuming the mixer outputs in microseconds, need to convert
	static constexpr uint32_t MIXER_OUTPUT_TO_CMD_SCALE = 1000;  //standard pwm
	//static constexpr uint32_t MIXER_OUTPUT_TO_CMD_SCALE = 125;   //oneshot125

	/* M0065 version info */
	static constexpr uint16_t VOXL2_IO_SW_VERSION = 2;
	static constexpr uint16_t VOXL2_IO_HW_VERSION = 35;  //this is the version of the HW
	int  _board_detect_retries{3};
	VOXL2_IO_EXTENDED_VERSION_INFO _version_info;

	/* Module update interval */
	static constexpr unsigned	_current_update_interval{4000}; // 250 Hz

	typedef struct {
		int32_t		baud_rate{VOXL2_IO_DEFAULT_BAUD};
		int32_t		pwm_min{0};
		int32_t		pwm_max{0};
		int32_t     pwm_dis{0};
		int32_t     pwm_cal_min{0};
		int32_t     pwm_cal_max{0};
		//int32_t		pwm_failsafe{VOXL2_IO_DEFAULT_FAILSAFE};
		//int32_t 	param_rc_input_proto{0};
		int32_t		param_rc_rssi_pwm_chan{0};
		int32_t		function_map[VOXL2_IO_OUTPUT_CHANNELS] {0, 0, 0, 0, 0, 0, 0, 0};
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

	char 				_device[10] {VOXL2_IO_DEFAULT_PORT};
	Serial				_uart_port{};

	/* Mixer output */
	MixingOutput 	_mixing_output;

	/* RC input */
	VOXL2_IOPacket _voxl2_io_packet;
	uint64_t _rc_last_valid_time;
	uint16_t _raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {UINT16_MAX};
	unsigned int _sbus_frame_drops{0};
	uint32_t _sbus_total_frames{0};

	/* Publications */
	uORB::PublicationMulti<input_rc_s> _rc_pub{ORB_ID(input_rc)};

	/* Subscriptions */
	uORB::Subscription 	_parameter_update_sub{ORB_ID(parameter_update)};

	bool		_pwm_on{false};
	bool		_outputs_disabled{false};

	perf_counter_t		_cycle_perf;
	perf_counter_t		_output_update_perf;

	bool 			_debug{false};

	static const uint8_t 	READ_BUF_SIZE = 128;
	uint8_t			_read_buf[READ_BUF_SIZE];
	uint32_t		_bytes_sent{0};
	uint32_t		_bytes_received{0};
	uint32_t		_packets_sent{0};
	uint32_t		_packets_received{0};

	int	load_params(voxl2_io_params_t *params);
	int update_params();
	int calibrate_escs();
	std::string board_id_to_name(int board_id);
};
