/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file rc_update.h
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

namespace rc_update
{

/**
 ** class RCUpdate
 *
 * Handling of RC updates
 */
class RCUpdate : public ModuleBase<RCUpdate>, public ModuleParams, public px4::WorkItem
{
public:
	RCUpdate();
	~RCUpdate() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

protected:
	static constexpr uint64_t VALID_DATA_MIN_INTERVAL_US{1_s / 3}; // assume valid RC input is at least 3 Hz

	void Run() override;

	/**
	 * Check for changes in rc_parameter_map
	 */
	void		rc_parameter_map_poll(bool forced = false);

	/**
	 * update the RC functions. Call this when the parameters change.
	 */
	void		update_rc_functions();

	void		UpdateManualControlInput(const hrt_abstime &timestamp_sample);
	void		UpdateManualSwitches(const hrt_abstime &timestamp_sample);

	/**
	 * Update our local parameter cache.
	 */
	void updateParams() override;

	/**
	 * Get and limit value for specified RC function. Returns NAN if not mapped.
	 */
	float		get_rc_value(uint8_t func, float min_value, float max_value) const;

	/**
	 * Get on/off switch position from the RC channel of the specified function, based on a single threshold.
	 *
	 * @param function according to rc_channels_s::FUNCTION_XXX
	 * @param threshold according to RC_XXX_TH parameters, negative means on and off are flipped
	 */
	switch_pos_t getRCSwitchOnOffPosition(uint8_t function, float threshold) const;

	/**
	 * Get on/off switch position from the RC channel of the specified function, based on a range.
	 *
	 * If BOTH thresholds are negative, on and off are flipped.
	 *
	 * @param function according to rc_channels_s::FUNCTION_XXX
	 * @param threshold_min according to RC_XXX_TH_MIN parameters
	 * @param threshold_max according to RC_XXX_TH_MAX parameters
	 */
	switch_pos_t getRCSwitchOnOffPositionFromRange(uint8_t function, float threshold_min, float threshold_max) const;

	/**
	 * Update parameters from RC channels if the functionality is activated and the
	 * input has changed since the last update
	 */
	void		set_params_from_rc();

	void		map_flight_modes_buttons();

	static constexpr uint8_t RC_MAX_CHAN_COUNT{input_rc_s::RC_INPUT_MAX_CHANNELS}; /**< maximum number of r/c channels we handle */

	struct Parameters {
		uint16_t min[RC_MAX_CHAN_COUNT];
		uint16_t trim[RC_MAX_CHAN_COUNT];
		uint16_t max[RC_MAX_CHAN_COUNT];
		uint16_t dz[RC_MAX_CHAN_COUNT];
		bool rev[RC_MAX_CHAN_COUNT];

		int32_t rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];
	} _parameters{};

	struct ParameterHandles {
		param_t min[RC_MAX_CHAN_COUNT];
		param_t trim[RC_MAX_CHAN_COUNT];
		param_t max[RC_MAX_CHAN_COUNT];
		param_t rev[RC_MAX_CHAN_COUNT];
		param_t dz[RC_MAX_CHAN_COUNT];

		param_t rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];
		param_t rc_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];	/**< param handles for the parameters which are bound
								to a RC channel, equivalent float values in the
								_parameters struct are not existing
								because these parameters are never read. */
	} _parameter_handles{};

	uORB::SubscriptionCallbackWorkItem _input_rc_sub{this, ORB_ID(input_rc)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _rc_parameter_map_sub{ORB_ID(rc_parameter_map)};

	uORB::Publication<rc_channels_s> _rc_channels_pub{ORB_ID(rc_channels)};
	uORB::PublicationMulti<manual_control_setpoint_s> _manual_control_input_pub{ORB_ID(manual_control_input)};
	uORB::Publication<manual_control_switches_s> _manual_control_switches_pub{ORB_ID(manual_control_switches)};

	manual_control_switches_s _manual_switches_previous{};
	manual_control_switches_s _manual_switches_last_publish{};
	rc_channels_s _rc{};
	bool _rc_calibrated{false};

	rc_parameter_map_s _rc_parameter_map {};
	float _param_rc_values[rc_parameter_map_s::RC_PARAM_MAP_NCHAN] {};	/**< parameter values for RC control */

	hrt_abstime _last_manual_control_input_publish{0};
	hrt_abstime _last_rc_to_param_map_time{0};
	hrt_abstime _last_timestamp_signal{0};

	uint16_t _rc_values_previous[RC_MAX_CHAN_COUNT] {};
	float _last_manual_control_input[3] {};
	bool _aux_already_active[3] = {false, false, false};

	uint8_t _channel_count_previous{0};
	uint8_t _input_source_previous{input_rc_s::RC_INPUT_SOURCE_UNKNOWN};

	uint8_t _potential_button_press_slot{0};
	systemlib::Hysteresis _button_pressed_hysteresis{false};
	systemlib::Hysteresis _rc_signal_lost_hysteresis{true};

	uint8_t _channel_count_max{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _valid_data_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": valid data interval")};

	DEFINE_PARAMETERS(

		(ParamInt<px4::params::RC_MAP_ROLL>) _param_rc_map_roll,
		(ParamInt<px4::params::RC_MAP_PITCH>) _param_rc_map_pitch,
		(ParamInt<px4::params::RC_MAP_YAW>) _param_rc_map_yaw,
		(ParamInt<px4::params::RC_MAP_THROTTLE>) _param_rc_map_throttle,
		(ParamInt<px4::params::RC_MAP_FAILSAFE>) _param_rc_map_failsafe,

		(ParamInt<px4::params::RC_MAP_FLTMODE>) _param_rc_map_fltmode,

		(ParamInt<px4::params::RC_MAP_FLAPS>) _param_rc_map_flaps,

		(ParamInt<px4::params::RC_MAP_RETURN_SW>) _param_rc_map_return_sw,
		(ParamInt<px4::params::RC_MAP_LOITER_SW>) _param_rc_map_loiter_sw,
		(ParamInt<px4::params::RC_MAP_OFFB_SW>) _param_rc_map_offb_sw,
		(ParamInt<px4::params::RC_MAP_KILL_SW>) _param_rc_map_kill_sw,
		(ParamInt<px4::params::RC_MAP_ARM_SW>) _param_rc_map_arm_sw,
		(ParamInt<px4::params::RC_MAP_TRANS_SW>) _param_rc_map_trans_sw,
		(ParamInt<px4::params::RC_MAP_GEAR_SW>) _param_rc_map_gear_sw,
		(ParamInt<px4::params::RC_MAP_FLTM_BTN>) _param_rc_map_fltm_btn,

		(ParamInt<px4::params::RC_MAP_AUX1>) _param_rc_map_aux1,
		(ParamInt<px4::params::RC_MAP_AUX2>) _param_rc_map_aux2,
		(ParamInt<px4::params::RC_MAP_AUX3>) _param_rc_map_aux3,
		(ParamInt<px4::params::RC_MAP_AUX4>) _param_rc_map_aux4,
		(ParamInt<px4::params::RC_MAP_AUX5>) _param_rc_map_aux5,
		(ParamInt<px4::params::RC_MAP_AUX6>) _param_rc_map_aux6,

		(ParamInt<px4::params::RC_MAP_ENG_MOT>) _param_rc_map_eng_mot,

		(ParamInt<px4::params::RC_FAILS_THR>) _param_rc_fails_thr,

		(ParamInt<px4::params::RC_MAP_PAY_SW>) _param_rc_map_payload_sw,

		(ParamFloat<px4::params::RC_LOITER_TH>) _param_rc_loiter_th,
		(ParamFloat<px4::params::RC_OFFB_TH>) _param_rc_offb_th,
		(ParamFloat<px4::params::RC_KILLSWITCH_TH>) _param_rc_killswitch_th,
		(ParamFloat<px4::params::RC_ARMSWITCH_TH>) _param_rc_armswitch_th,
		(ParamFloat<px4::params::RC_TRANS_TH>) _param_rc_trans_th,
		(ParamFloat<px4::params::RC_GEAR_TH>) _param_rc_gear_th,
		(ParamFloat<px4::params::RC_RETURN_TH_MIN>) _param_rc_return_th_min,
		(ParamFloat<px4::params::RC_RETURN_TH_MAX>) _param_rc_return_th_max,
		(ParamFloat<px4::params::RC_ENG_MOT_TH>) _param_rc_eng_mot_th,
		(ParamFloat<px4::params::RC_PAYLOAD_TH>) _param_rc_payload_th,

		(ParamInt<px4::params::RC_CHAN_CNT>) _param_rc_chan_cnt
	)
};
} /* namespace rc_update */
