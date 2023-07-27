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

/**
 * @file rc_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "rc_update.h"

using namespace time_literals;

namespace rc_update
{

// TODO: find a better home for this
static bool operator ==(const manual_control_switches_s &a, const manual_control_switches_s &b)
{
	return (a.mode_slot == b.mode_slot &&
		a.return_switch == b.return_switch &&
		a.loiter_switch == b.loiter_switch &&
		a.offboard_switch == b.offboard_switch &&
		a.kill_switch == b.kill_switch &&
		a.arm_switch == b.arm_switch &&
		a.transition_switch == b.transition_switch &&
		a.gear_switch == b.gear_switch &&
		a.photo_switch == b.photo_switch &&
		a.video_switch == b.video_switch &&
		a.engage_main_motor_switch == b.engage_main_motor_switch);
}

static bool operator !=(const manual_control_switches_s &a, const manual_control_switches_s &b) { return !(a == b); }


RCUpdate::RCUpdate() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// initialize parameter handles
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		char nbuf[16];

		/* min values */
		snprintf(nbuf, sizeof(nbuf), "RC%d_MIN", i + 1);
		_parameter_handles.min[i] = param_find(nbuf);

		/* trim values */
		snprintf(nbuf, sizeof(nbuf), "RC%d_TRIM", i + 1);
		_parameter_handles.trim[i] = param_find(nbuf);

		/* max values */
		snprintf(nbuf, sizeof(nbuf), "RC%d_MAX", i + 1);
		_parameter_handles.max[i] = param_find(nbuf);

		/* channel reverse */
		snprintf(nbuf, sizeof(nbuf), "RC%d_REV", i + 1);
		_parameter_handles.rev[i] = param_find(nbuf);

		/* channel deadzone */
		snprintf(nbuf, sizeof(nbuf), "RC%d_DZ", i + 1);
		_parameter_handles.dz[i] = param_find(nbuf);
	}

	// RC to parameter mapping for changing parameters with RC
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		// shifted by 1 because param name starts at 1
		char name[rc_parameter_map_s::PARAM_ID_LEN];
		snprintf(name, rc_parameter_map_s::PARAM_ID_LEN, "RC_MAP_PARAM%d", i + 1);
		_parameter_handles.rc_map_param[i] = param_find(name);
	}

	rc_parameter_map_poll(true /* forced */);
	updateParams(); // Call is needed to populate the _rc.function array

	_button_pressed_hysteresis.set_hysteresis_time_from(false, 50_ms);
}

RCUpdate::~RCUpdate()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_valid_data_interval_perf);
}

bool RCUpdate::init()
{
	if (!_input_rc_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void RCUpdate::updateParams()
{
	ModuleParams::updateParams();

	// rc values
	for (unsigned int i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		float min = 0.f;
		param_get(_parameter_handles.min[i], &min);
		_parameters.min[i] = min;

		float trim = 0.f;
		param_get(_parameter_handles.trim[i], &trim);
		_parameters.trim[i] = trim;

		float max = 0.f;
		param_get(_parameter_handles.max[i], &max);
		_parameters.max[i] = max;

		float rev = 0.f;
		param_get(_parameter_handles.rev[i], &rev);
		_parameters.rev[i] = (rev < 0.f);

		float dz = 0.f;
		param_get(_parameter_handles.dz[i], &dz);
		_parameters.dz[i] = dz;
	}

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		param_get(_parameter_handles.rc_map_param[i], &(_parameters.rc_map_param[i]));
	}

	update_rc_functions();

	_rc_calibrated = _param_rc_chan_cnt.get() > 0
			 && (_param_rc_map_throttle.get() > 0
			     || _param_rc_map_roll.get() > 0
			     || _param_rc_map_pitch.get() > 0
			     || _param_rc_map_yaw.get() > 0);

	// deprecated parameters, will be removed post v1.12 once QGC is updated
	{
		int32_t rc_map_value = 0;

		if (param_get(param_find("RC_MAP_MODE_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_MODE_SW deprecated");
				param_reset(param_find("RC_MAP_MODE_SW"));
			}
		}

		if (param_get(param_find("RC_MAP_RATT_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_RATT_SW deprecated");
				param_reset(param_find("RC_MAP_RATT_SW"));
			}
		}

		if (param_get(param_find("RC_MAP_POSCTL_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_POSCTL_SW deprecated");
				param_reset(param_find("RC_MAP_POSCTL_SW"));
			}
		}

		if (param_get(param_find("RC_MAP_ACRO_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_ACRO_SW deprecated");
				param_reset(param_find("RC_MAP_ACRO_SW"));
			}
		}

		if (param_get(param_find("RC_MAP_STAB_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_STAB_SW deprecated");
				param_reset(param_find("RC_MAP_STAB_SW"));
			}
		}

		if (param_get(param_find("RC_MAP_MAN_SW"), &rc_map_value) == PX4_OK) {
			if (rc_map_value != 0) {
				PX4_WARN("RC_MAP_MAN_SW deprecated");
				param_reset(param_find("RC_MAP_MAN_SW"));
			}
		}
	}

	// Center throttle trim when it's set to the minimum to correct for hardcoded QGC RC calibration
	// See https://github.com/mavlink/qgroundcontrol/commit/0577af2e944a0f53919aeb1367d580f744004b2c
	const int8_t throttle_channel = _rc.function[rc_channels_s::FUNCTION_THROTTLE];

	if (throttle_channel >= 0 && throttle_channel < RC_MAX_CHAN_COUNT) {
		const uint16_t throttle_min = _parameters.min[throttle_channel];
		const uint16_t throttle_trim = _parameters.trim[throttle_channel];
		const uint16_t throttle_max = _parameters.max[throttle_channel];
		const bool throttle_rev = _parameters.rev[throttle_channel];

		const bool normal_case = !throttle_rev && (throttle_trim == throttle_min);
		const bool reversed_case = throttle_rev && (throttle_trim == throttle_max);

		if (normal_case || reversed_case) {
			const uint16_t new_throttle_trim = (throttle_min + throttle_max) / 2;
			_parameters.trim[throttle_channel] = new_throttle_trim;
		}
	}
}

void RCUpdate::update_rc_functions()
{
	/* update RC function mappings */
	_rc.function[rc_channels_s::FUNCTION_THROTTLE] = _param_rc_map_throttle.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_ROLL] = _param_rc_map_roll.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_PITCH] = _param_rc_map_pitch.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_YAW] = _param_rc_map_yaw.get() - 1;

	_rc.function[rc_channels_s::FUNCTION_RETURN] = _param_rc_map_return_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_LOITER] = _param_rc_map_loiter_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_OFFBOARD] = _param_rc_map_offb_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_KILLSWITCH] = _param_rc_map_kill_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_ARMSWITCH] = _param_rc_map_arm_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_TRANSITION] = _param_rc_map_trans_sw.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_GEAR] = _param_rc_map_gear_sw.get() - 1;

	_rc.function[rc_channels_s::FUNCTION_FLAPS] = _param_rc_map_flaps.get() - 1;

	_rc.function[rc_channels_s::FUNCTION_AUX_1] = _param_rc_map_aux1.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_AUX_2] = _param_rc_map_aux2.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_AUX_3] = _param_rc_map_aux3.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_AUX_4] = _param_rc_map_aux4.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_AUX_5] = _param_rc_map_aux5.get() - 1;
	_rc.function[rc_channels_s::FUNCTION_AUX_6] = _param_rc_map_aux6.get() - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}

	_rc.function[rc_channels_s::FUNCTION_ENGAGE_MAIN_MOTOR] = _param_rc_map_eng_mot.get() - 1;

	map_flight_modes_buttons();
}

void RCUpdate::rc_parameter_map_poll(bool forced)
{
	if (_rc_parameter_map_sub.updated() || forced) {
		_rc_parameter_map_sub.copy(&_rc_parameter_map);

		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				_parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				_parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}
		}

		PX4_DEBUG("rc to parameter map updated");

		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
				  i,
				  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
				  (double)_rc_parameter_map.scale[i],
				  (double)_rc_parameter_map.value0[i],
				  (double)_rc_parameter_map.value_min[i],
				  (double)_rc_parameter_map.value_max[i]
				 );
		}
	}
}

float RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value) const
{
	if (_rc.function[func] >= 0) {
		return math::constrain(_rc.channels[_rc.function[func]], min_value, max_value);
	}

	return 0.f;
}

void RCUpdate::set_params_from_rc()
{
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::FUNCTION_PARAM_1 + i), -1.f, 1.f);

		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);

			param_set(_parameter_handles.rc_param[i], &param_val);
		}
	}
}

void
RCUpdate::map_flight_modes_buttons()
{
	static_assert(rc_channels_s::FUNCTION_FLTBTN_SLOT_1 + manual_control_switches_s::MODE_SLOT_NUM <= sizeof(
			      _rc.function) / sizeof(_rc.function[0]), "Unexpected number of RC functions");
	static_assert(rc_channels_s::FUNCTION_FLTBTN_SLOT_COUNT == manual_control_switches_s::MODE_SLOT_NUM,
		      "Unexpected number of Flight Modes slots");

	// Reset all the slots to -1
	for (uint8_t slot = 0; slot < manual_control_switches_s::MODE_SLOT_NUM; slot++) {
		_rc.function[rc_channels_s::FUNCTION_FLTBTN_SLOT_1 + slot] = -1;
	}

	// If the functionality is disabled we don't need to map channels
	const int flightmode_buttons = _param_rc_map_fltm_btn.get();

	if (flightmode_buttons == 0) {
		return;
	}

	uint8_t slot = 0;

	for (uint8_t channel = 0; channel < RC_MAX_CHAN_COUNT; channel++) {
		if (flightmode_buttons & (1 << channel)) {
			PX4_DEBUG("Slot %d assigned to channel %d", slot + 1, channel);
			_rc.function[rc_channels_s::FUNCTION_FLTBTN_SLOT_1 + slot] = channel;
			slot++;
		}

		if (slot >= manual_control_switches_s::MODE_SLOT_NUM) {
			// we have filled all the available slots
			break;
		}
	}
}

void RCUpdate::Run()
{
	if (should_exit()) {
		_input_rc_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	rc_parameter_map_poll();

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	input_rc_s input_rc;

	if (_input_rc_sub.update(&input_rc)) {

		// warn if the channel count is changing (possibly indication of error)
		if (!input_rc.rc_lost) {
			if ((_channel_count_previous != input_rc.channel_count)
			    && (_channel_count_previous > 0)) {
				PX4_WARN("channel count changed %d -> %d", _channel_count_previous, input_rc.channel_count);
			}

			if ((_input_source_previous != input_rc.input_source)
			    && (_input_source_previous != input_rc_s::RC_INPUT_SOURCE_UNKNOWN)) {
				PX4_WARN("input source changed %d -> %d", _input_source_previous, input_rc.input_source);
			}
		}

		const bool input_source_stable = (input_rc.input_source == _input_source_previous);
		const bool channel_count_stable = (input_rc.channel_count == _channel_count_previous);

		_input_source_previous = input_rc.input_source;
		_channel_count_previous = input_rc.channel_count;

		const uint8_t channel_count_limited = math::min(input_rc.channel_count, RC_MAX_CHAN_COUNT);

		if (channel_count_limited > _channel_count_max) {
			_channel_count_max = channel_count_limited;
		}

		/* detect RC signal loss */
		bool signal_lost = true;

		/* check flags and require at least four channels to consider the signal valid */
		if (input_rc.rc_lost || input_rc.rc_failsafe || input_rc.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;

		} else if ((input_rc.input_source == input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM ||
			    input_rc.input_source == input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM)
			   && input_rc.channel_count == 16) {

			// This is a specific RC lost check for RFD 868+/900 Modems on PPM.
			// The observation was that when RC is lost, 16 channels are active and the first 12 are 1000
			// and the remaining ones are 0.
			for (unsigned int i = 0; i < 16; i++) {
				if (i < 12 && input_rc.values[i] > 999 && input_rc.values[i] < 1005) {
					signal_lost = true;

				} else if (input_rc.values[i] == 0) {
					signal_lost = true;

				} else {
					signal_lost = false;
					break;
				}
			}

		} else {
			/* signal looks good */
			signal_lost = false;

			/* check failsafe */
			int8_t fs_ch = _rc.function[_param_rc_map_failsafe.get()]; // get channel mapped to throttle

			if (_param_rc_map_failsafe.get() > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _param_rc_map_failsafe.get() - 1;
			}

			if (_param_rc_fails_thr.get() > 0 && fs_ch >= 0) {
				/* failsafe configured */
				if ((_param_rc_fails_thr.get() < _parameters.min[fs_ch] && input_rc.values[fs_ch] < _param_rc_fails_thr.get()) ||
				    (_param_rc_fails_thr.get() > _parameters.max[fs_ch] && input_rc.values[fs_ch] > _param_rc_fails_thr.get())) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		/* read out and scale values from raw message even if signal is invalid */
		for (unsigned int i = 0; i < channel_count_limited; i++) {
			// float conversions of uint16_t values
			const float value = input_rc.values[i];
			const float min = _parameters.min[i];
			const float trim = _parameters.trim[i];
			const float max = _parameters.max[i];
			const float dz = _parameters.dz[i];

			// piecewise linear function to apply RC calibration
			_rc.channels[i] = math::interpolateNXY(value,
			{min, trim - dz, trim + dz, max},
			{-1.f, 0.f, 0.f, 1.f});

			if (_parameters.rev[i]) {
				_rc.channels[i] = -_rc.channels[i];
			}

			/* handle any parameter-induced blowups */
			if (!PX4_ISFINITE(_rc.channels[i])) {
				_rc.channels[i] = 0.f;
			}
		}

		/*
		 * some RC systems glitch after a reboot, we should ignore the first 100ms of regained signal
		 * as the glitch might be interpreted as a commanded stick action or a flight mode switch
		 */
		_rc_signal_lost_hysteresis.set_hysteresis_time_from(true, 100_ms);
		_rc_signal_lost_hysteresis.set_state_and_update(signal_lost, hrt_absolute_time());

		_rc.channel_count = input_rc.channel_count;
		_rc.rssi = input_rc.rssi;
		_rc.signal_lost = _rc_signal_lost_hysteresis.get_state();
		_rc.timestamp = input_rc.timestamp_last_signal;
		_rc.frame_drop_count = input_rc.rc_lost_frame_count;

		/* publish rc_channels topic even if signal is invalid, for debug */
		_rc_channels_pub.publish(_rc);

		// only publish manual control if the signal is present and regularly updating
		if (input_source_stable && channel_count_stable && !_rc_signal_lost_hysteresis.get_state()) {

			if ((input_rc.timestamp_last_signal > _last_timestamp_signal)
			    && (input_rc.timestamp_last_signal < _last_timestamp_signal + VALID_DATA_MIN_INTERVAL_US)) {

				perf_count(_valid_data_interval_perf);

				// check if channels actually updated
				bool rc_updated = false;

				for (unsigned i = 0; i < channel_count_limited; i++) {
					if (_rc_values_previous[i] != input_rc.values[i]) {
						rc_updated = true;
						break;
					}
				}

				// limit processing if there's no update
				if (rc_updated || (hrt_elapsed_time(&_last_manual_control_input_publish) > 300_ms)) {
					UpdateManualControlInput(input_rc.timestamp_last_signal);
				}

				UpdateManualSwitches(input_rc.timestamp_last_signal);

				/* Update parameters from RC Channels (tuning with RC) if activated */
				if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1_s) {
					set_params_from_rc();
					_last_rc_to_param_map_time = hrt_absolute_time();
				}
			}

			_last_timestamp_signal = input_rc.timestamp_last_signal;

		} else {
			// RC input unstable or lost, clear any previous manual_switches
			if (_manual_switches_last_publish.timestamp_sample != 0) {
				_manual_switches_last_publish = {};
			}
		}

		memcpy(_rc_values_previous, input_rc.values, sizeof(input_rc.values[0]) * channel_count_limited);
		static_assert(sizeof(_rc_values_previous) == sizeof(input_rc.values), "check sizeof(_rc_values_previous)");
	}

	perf_end(_loop_perf);
}

switch_pos_t RCUpdate::getRCSwitchOnOffPosition(uint8_t function, float threshold) const
{
	if (_rc.function[function] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[function]] + 0.5f; // Rescale [-1,1] -> [0,1] range

		if (threshold < 0.f) {
			value = -value;
		}

		return (value > threshold) ? manual_control_switches_s::SWITCH_POS_ON : manual_control_switches_s::SWITCH_POS_OFF;
	}

	return manual_control_switches_s::SWITCH_POS_NONE;
}

void RCUpdate::UpdateManualSwitches(const hrt_abstime &timestamp_sample)
{
	manual_control_switches_s switches{};
	switches.timestamp_sample = timestamp_sample;

	// check mode slot (RC_MAP_FLTMODE)
	if (_param_rc_map_fltmode.get() > 0) {
		// number of valid slots
		static constexpr int num_slots = manual_control_switches_s::MODE_SLOT_NUM;

		// the half width of the range of a slot is the total range
		// divided by the number of slots, again divided by two
		static constexpr float slot_width_half = 1.f / num_slots;

		// min is -1, max is +1, range is 2. We offset below min and max
		static constexpr float slot_min = -1.f - 0.05f;
		static constexpr float slot_max =  1.f + 0.05f;

		// the slot gets mapped by first normalizing into a 0..1 interval using min
		// and max. Then the right slot is obtained by multiplying with the number of
		// slots. And finally we add half a slot width to ensure that integer rounding
		// will take us to the correct final index.
		const float value = _rc.channels[_param_rc_map_fltmode.get() - 1];
		switches.mode_slot = (((((value - slot_min) * num_slots) + slot_width_half) / (slot_max - slot_min)) +
				      slot_width_half) + 1;

		if (switches.mode_slot > num_slots) {
			switches.mode_slot = num_slots;
		}

	} else if (_param_rc_map_fltm_btn.get() > 0) {
		switches.mode_slot = manual_control_switches_s::MODE_SLOT_NONE;
		bool is_consistent_button_press = false;

		for (uint8_t slot = 0; slot < manual_control_switches_s::MODE_SLOT_NUM; slot++) {

			// If the slot is not in use (-1), get_rc_value() will return 0
			float value = get_rc_value(rc_channels_s::FUNCTION_FLTBTN_SLOT_1 + slot, -1.0, 1.0);

			// The range goes from -1 to 1, checking that value is greater than 0.5f
			// corresponds to check that the signal is above 75% of the overall range.
			if (value > 0.5f) {
				const uint8_t current_button_press_slot = slot + 1;

				// The same button stays pressed consistently
				if (current_button_press_slot == _potential_button_press_slot) {
					is_consistent_button_press = true;
				}

				_potential_button_press_slot = current_button_press_slot;
				break;
			}
		}

		_button_pressed_hysteresis.set_state_and_update(is_consistent_button_press, timestamp_sample);

		if (_button_pressed_hysteresis.get_state()) {
			switches.mode_slot = _potential_button_press_slot;
		}
	}

	switches.return_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_RETURN, _param_rc_return_th.get());
	switches.loiter_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_LOITER, _param_rc_loiter_th.get());
	switches.offboard_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_OFFBOARD, _param_rc_offb_th.get());
	switches.kill_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_KILLSWITCH, _param_rc_killswitch_th.get());
	switches.arm_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_ARMSWITCH, _param_rc_armswitch_th.get());
	switches.transition_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_TRANSITION, _param_rc_trans_th.get());
	switches.gear_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_GEAR, _param_rc_gear_th.get());
	switches.engage_main_motor_switch =
		getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_ENGAGE_MAIN_MOTOR, _param_rc_eng_mot_th.get());
#if defined(ATL_MANTIS_RC_INPUT_HACKS)
	switches.photo_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_AUX_3, 0.5f);
	switches.video_switch = getRCSwitchOnOffPosition(rc_channels_s::FUNCTION_AUX_4, 0.5f);
#endif

	// last 2 switch updates identical within 1 second (simple protection from bad RC data)
	if ((switches == _manual_switches_previous)
	    && (switches.timestamp_sample < _manual_switches_previous.timestamp_sample + VALID_DATA_MIN_INTERVAL_US)) {

		const bool switches_changed = (switches != _manual_switches_last_publish);

		// publish immediately on change or at ~1 Hz
		if (switches_changed || (hrt_elapsed_time(&_manual_switches_last_publish.timestamp) >= 1_s)) {
			uint32_t switch_changes = _manual_switches_last_publish.switch_changes;

			if (switches_changed) {
				switch_changes++;
			}

			_manual_switches_last_publish = switches;
			_manual_switches_last_publish.switch_changes = switch_changes;
			_manual_switches_last_publish.timestamp_sample = _manual_switches_previous.timestamp_sample;
			_manual_switches_last_publish.timestamp = hrt_absolute_time();
			_manual_control_switches_pub.publish(_manual_switches_last_publish);
		}
	}

	_manual_switches_previous = switches;
}

void RCUpdate::UpdateManualControlInput(const hrt_abstime &timestamp_sample)
{
	manual_control_setpoint_s manual_control_input{};
	manual_control_input.timestamp_sample = timestamp_sample;
	manual_control_input.data_source = manual_control_setpoint_s::SOURCE_RC;

	// limit controls
	manual_control_input.roll = get_rc_value(rc_channels_s::FUNCTION_ROLL,    -1.f, 1.f);
	manual_control_input.pitch = get_rc_value(rc_channels_s::FUNCTION_PITCH,   -1.f, 1.f);
	manual_control_input.yaw = get_rc_value(rc_channels_s::FUNCTION_YAW,     -1.f, 1.f);
	manual_control_input.throttle = get_rc_value(rc_channels_s::FUNCTION_THROTTLE, -1.f, 1.f);
	manual_control_input.flaps = get_rc_value(rc_channels_s::FUNCTION_FLAPS,   -1.f, 1.f);
	manual_control_input.aux1  = get_rc_value(rc_channels_s::FUNCTION_AUX_1,   -1.f, 1.f);
	manual_control_input.aux2  = get_rc_value(rc_channels_s::FUNCTION_AUX_2,   -1.f, 1.f);
	manual_control_input.aux3  = get_rc_value(rc_channels_s::FUNCTION_AUX_3,   -1.f, 1.f);
	manual_control_input.aux4  = get_rc_value(rc_channels_s::FUNCTION_AUX_4,   -1.f, 1.f);
	manual_control_input.aux5  = get_rc_value(rc_channels_s::FUNCTION_AUX_5,   -1.f, 1.f);
	manual_control_input.aux6  = get_rc_value(rc_channels_s::FUNCTION_AUX_6,   -1.f, 1.f);
	manual_control_input.valid = _rc_calibrated;

	// publish manual_control_input topic
	manual_control_input.timestamp = hrt_absolute_time();
	_manual_control_input_pub.publish(manual_control_input);
	_last_manual_control_input_publish = manual_control_input.timestamp;
}

int RCUpdate::task_spawn(int argc, char *argv[])
{
	RCUpdate *instance = new RCUpdate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RCUpdate::print_status()
{
	PX4_INFO_RAW("Running\n");

	if (_channel_count_max > 0) {
		PX4_INFO_RAW(" #  MIN  MAX TRIM  DZ REV\n");

		for (int i = 0; i < _channel_count_max; i++) {
			PX4_INFO_RAW("%2d %4d %4d %4d %3d %3d\n", i, _parameters.min[i], _parameters.max[i], _parameters.trim[i],
				     _parameters.dz[i], _parameters.rev[i]);
		}
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_valid_data_interval_perf);

	return 0;
}

int RCUpdate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RCUpdate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches
and then publish as `rc_channels` and `manual_control_input`.

### Implementation
To reduce control latency, the module is scheduled on input_rc publications.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_update", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace rc_update

extern "C" __EXPORT int rc_update_main(int argc, char *argv[])
{
	return rc_update::RCUpdate::main(argc, argv);
}
