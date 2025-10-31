/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_dshot.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "DShotCommon.h"
#include "DShotTelemetry.h"

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

static constexpr int DSHOT_MAXIMUM_CHANNELS = esc_status_s::CONNECTED_ESC_MAX;

static constexpr hrt_abstime ESC_INIT_TELEM_WAIT_TIME = 3_s;


/** Dshot PWM frequency, Hz */
static constexpr unsigned int DSHOT150  =  150000u;
static constexpr unsigned int DSHOT300  =  300000u;
static constexpr unsigned int DSHOT600  =  600000u;

static constexpr uint16_t DSHOT_DISARM_VALUE = 0;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 1;
static constexpr uint16_t DSHOT_MAX_THROTTLE = 1999;

// We do this to avoid bringing in mavlink.h
// #include <mavlink/common/mavlink.h>
#define ACTUATOR_CONFIGURATION_BEEP 1
#define ACTUATOR_CONFIGURATION_3D_MODE_OFF 2
#define ACTUATOR_CONFIGURATION_3D_MODE_ON 3
#define ACTUATOR_CONFIGURATION_SPIN_DIRECTION1 4
#define ACTUATOR_CONFIGURATION_SPIN_DIRECTION2 5
#define ACTUATOR_CONFIGURATION_READ_SETTINGS 6
#define ACTUATOR_CONFIGURATION_WRITE_SETTING 7

class DShot final : public ModuleBase<DShot>, public OutputModuleInterface
{
public:
	DShot();
	~DShot() override;

	// @see ModuleBase
	static int custom_command(int argc, char *argv[]);

	// @see ModuleBase
	int print_status() override;

	// @see ModuleBase
	static int print_usage(const char *reason = nullptr);

	// @see ModuleBase
	static int task_spawn(int argc, char *argv[]);

	int init();

	void mixerChanged() override;

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	struct Command {
		uint16_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool save{false};
		bool expect_response{false};

		bool valid() const { return num_repetitions > 0; }
		void clear()
		{
			command = 0;
			num_repetitions = 0;
			motor_mask = 0;
			save = 0;
			expect_response = 0;
		}
	};

	bool initialize_dshot();

	void init_telemetry(const char *device, bool swap_rxtx);

	// Returns true when the telemetry index has wrapped, indicating all configured motors have been sampled.
	bool set_next_telemetry_index();

	void consume_esc_data(const EscData &data, TelemetrySource source);

	bool process_bdshot_erpm();


	void Run() override;

	void update_params();

	void handle_vehicle_commands();

	uint16_t convert_output_to_3d_scaling(uint16_t output);

	void handle_programming_sequence_state();

	// Mixer
	MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uint32_t _output_mask{0}; // Configured outputs for this (shouldn't this live in OutputModuleInterface?)

	// uORb
	esc_status_s _esc_status{};
	uORB::PublicationMultiData<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	// Status information
	uint32_t _bdshot_telem_online_mask = 0; // Mask indicating telem receive status for bidirectional dshot telem
	uint32_t _serial_telem_online_mask = 0; // Mask indicating telem receive status for serial telem
	uint16_t _esc_status_counter = 0;

	// Serial Telemetry
	DShotTelemetry _telemetry;
	static char _telemetry_device[20];
	static bool _telemetry_swap_rxtx;
	static px4::atomic_bool _request_telemetry_init;
	int _telemetry_motor_index = 0;
	uint32_t _telemetry_requested_mask = 0;
	hrt_abstime _telem_delay_until = ESC_INIT_TELEM_WAIT_TIME;

	uint32_t _serial_telem_errors[DSHOT_MAXIMUM_CHANNELS] = {};
	uint32_t _bdshot_telem_errors[DSHOT_MAXIMUM_CHANNELS] = {};

	// Perf counters
	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_bdshot_success_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot success")};
	perf_counter_t	_bdshot_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot error")};
	perf_counter_t	_bdshot_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot timeout")};
	perf_counter_t	_telem_success_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem success")};
	perf_counter_t	_telem_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem error")};
	perf_counter_t	_telem_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem timeout")};
	perf_counter_t	_telem_allsampled_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem all sampled")};

	// Commands
	Command _current_command{};

	enum class ProgrammingState {
		Idle,
		EnterMode,
		SendAddress,
		SendValue,
		ExitMode,
		Save
	};

	ProgrammingState _programming_state{ProgrammingState::Idle};

	uint16_t _programming_address{};
	uint16_t _programming_value{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamBool<px4::params::DSHOT_3D_ENABLE>) _param_dshot_3d_enable,
		(ParamInt<px4::params::DSHOT_3D_DEAD_H>) _param_dshot_3d_dead_h,
		(ParamInt<px4::params::DSHOT_3D_DEAD_L>) _param_dshot_3d_dead_l,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count,
		(ParamBool<px4::params::DSHOT_BIDIR_EN>) _param_dshot_bidir_en,
		(ParamBool<px4::params::DSHOT_TEL_CFG>) _param_dshot_tel_cfg
	)
};
