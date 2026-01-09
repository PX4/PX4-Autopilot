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
#pragma once

#include <drivers/drv_dshot.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/esc_eeprom_write.h>

#include "DShotCommon.h"
#include "DShotTelemetry.h"

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

static constexpr hrt_abstime ESC_INIT_TELEM_DELAY = 5_s;

/// Dshot PWM frequency (Hz)
static constexpr uint32_t DSHOT150 = 150000u;
static constexpr uint32_t DSHOT300 = 300000u;
static constexpr uint32_t DSHOT600 = 600000u;

static constexpr uint16_t DSHOT_DISARM_VALUE = 0;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 1;
static constexpr uint16_t DSHOT_MAX_THROTTLE = 1999;

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

	bool updateOutputs(uint16_t *outputs, unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	enum class State {
		Disarmed,
		Armed
	} _state = State::Disarmed;

	// Disallow copy construction and move assignment
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	bool initialize_dshot();
	void init_telemetry(const char *device, bool swap_rxtx);

	uint8_t esc_armed_mask(uint16_t *outputs, uint8_t num_outputs);

	void update_motor_outputs(uint16_t *outputs, int num_outputs);
	void update_motor_commands(int num_outputs);
	void select_next_command();

	bool set_next_telemetry_index(); // Returns true when the telemetry index has wrapped, indicating all configured motors have been sampled.
	bool process_serial_telemetry();
	bool process_bdshot_telemetry();

	void consume_esc_data(const EscData &data, TelemetrySource source);

	uint16_t calculate_output_value(uint16_t raw, int index);
	uint16_t convert_output_to_3d_scaling(uint16_t output);

	void Run() override;
	void update_params();

	// Mavlink command handlers
	void handle_vehicle_commands();
	void handle_configure_actuator(const vehicle_command_s &command);
	void handle_esc_request_eeprom(const vehicle_command_s &command);

	// Mixer
	MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	// Actuator-order masks (indexed by output channel)
	uint32_t _output_mask{0};
	uint32_t _bdshot_output_mask{0};

	// Motor-order masks (indexed by motor number: Motor1=0, Motor2=1, etc.)
	uint32_t _motor_mask{0};
	uint32_t _bdshot_motor_mask{0};

	// Motor-to-channel mapping: _motor_to_channel[motor_index] = actuator_channel (-1 if not assigned)
	int8_t _motor_to_channel[DSHOT_MAXIMUM_CHANNELS] = {-1, -1, -1, -1, -1, -1, -1, -1};

	// uORB
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _esc_eeprom_write_sub{ORB_ID(esc_eeprom_write)};

	uORB::PublicationMultiData<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	esc_status_s _esc_status{};

	// Status information
	uint32_t _bdshot_telem_online_mask = 0; // Mask indicating telem receive status for bidirectional dshot telem
	uint32_t _serial_telem_online_mask = 0; // Mask indicating telem receive status for serial telem
	uint32_t _serial_telem_errors[DSHOT_MAXIMUM_CHANNELS] = {};
	uint32_t _bdshot_telem_errors[DSHOT_MAXIMUM_CHANNELS] = {};
	uint8_t _bdshot_edt_requested_mask = 0;
	uint8_t _settings_requested_mask = 0;

	// Array of timestamps indicating when the telemetry came online
	hrt_abstime _serial_telem_online_timestamps[DSHOT_MAXIMUM_CHANNELS] = {};
	hrt_abstime _bdshot_telem_online_timestamps[DSHOT_MAXIMUM_CHANNELS] = {};

	// Serial Telemetry
	DShotTelemetry _telemetry;
	static char _telemetry_device[20];
	static bool _telemetry_swap_rxtx;
	static px4::atomic_bool _request_telemetry_init;
	int _telemetry_motor_index = 0;
	uint32_t _telemetry_requested_mask = 0;
	hrt_abstime _serial_telem_delay_until = ESC_INIT_TELEM_DELAY;

	// Parameters we must load only at init
	bool _serial_telemetry_enabled = false;
	bool _bdshot_edt_enabled = false;

	// Hardware initialization state
	bool _hardware_initialized = false;
	uint32_t _dshot_frequency = 0;
	uint32_t _bdshot_timer_channels = 0;

	// Perf counters
	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_bdshot_success_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot success")};
	perf_counter_t	_bdshot_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot error")};
	perf_counter_t	_telem_success_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem success")};
	perf_counter_t	_telem_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem error")};
	perf_counter_t	_telem_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem timeout")};
	perf_counter_t	_telem_allsampled_perf{perf_alloc(PC_COUNT, MODULE_NAME": telem all sampled")};

	// Commands
	struct DShotCommand {
		uint16_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool save{false};
		bool expect_response{false};

		bool finished() const { return num_repetitions == 0; }
		void clear()
		{
			command = 0;
			num_repetitions = 0;
			motor_mask = 0;
			save = 0;
			expect_response = 0;
		}
	};

	DShotCommand _current_command{};

	// DShot Programming Mode
	enum class ProgrammingState {
		Idle,
		EnterMode,
		SendAddress,
		SendValue,
		ExitMode
	};

	esc_eeprom_write_s _esc_eeprom_write{};
	bool _dshot_programming_active = {};
	uint32_t _settings_written_mask[2] = {};

	ProgrammingState _programming_state{ProgrammingState::Idle};

	uint16_t _programming_address{};
	uint16_t _programming_value{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DSHOT_ESC_TYPE>) _param_dshot_esc_type,
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamBool<px4::params::DSHOT_3D_ENABLE>) _param_dshot_3d_enable,
		(ParamInt<px4::params::DSHOT_3D_DEAD_H>) _param_dshot_3d_dead_h,
		(ParamInt<px4::params::DSHOT_3D_DEAD_L>) _param_dshot_3d_dead_l,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count,
		(ParamBool<px4::params::DSHOT_BIDIR_EDT>) _param_dshot_bidir_edt,
		(ParamBool<px4::params::DSHOT_TEL_CFG>) _param_dshot_tel_cfg
	)
};
