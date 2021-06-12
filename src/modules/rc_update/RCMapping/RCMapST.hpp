/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file RCMap.hpp
 *
 * Base class for similarities of Yuneec STXX remotes.
 *
 * @author Dennis Mannhart <dennis@yuneecresearch.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMap.hpp"

#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace time_literals;

namespace sensors
{

class Button
{

public:
	Button(bool init_state, bool trigger_high = false):
		_state(init_state),
		_prev_pushed(init_state),
		_trigger_high(trigger_high)
	{
	}

	~Button() {}

	void update_state(const bool pushed)
	{
		if (_trigger_high) {
			// button was pushed when going from false (low) to true (high)
			if (!_prev_pushed && pushed) {
				_state = !_state;
			}

		} else {
			// button was pushed when going from true (high) to false (low)
			if (_prev_pushed && !pushed) {
				_state = !_state;
			}
		}

		_prev_pushed = pushed;
	}

	bool get_state()
	{
		return _state;
	}

	void reset_button()
	{
		_state = _trigger_high;
		_prev_pushed = _state;
	}



private:
	bool _state;
	bool _prev_pushed;
	bool _trigger_high;
};

class RCMapST : public RCMap
{
public:
	RCMapST() = default;
	virtual ~RCMapST() = default;

	enum class Error : int {
		None = 0,
		Version
	};

	/**
	 * Map STXX common data (channels) to the manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	virtual int map(manual_control_setpoint_s &man, const input_rc_s &input_rc, int32_t &parameters)
	{
		// map all none-switch/trim channels to range [-1,1]
		man.z = unit_range(input_rc.values[CHANNEL_LEFT_STICK_UP]); // thrust (mode 2)
		man.r = unit_range(input_rc.values[CHANNEL_LEFT_STICK_RIGHT]); // yaw (mode 2)
		man.x = unit_range(input_rc.values[CHANNEL_RIGHT_STICK_UP]); // pitch (mode 2)
		man.y = unit_range(input_rc.values[CHANNEL_RIGHT_STICK_RIGHT]); // roll (mode 2)


		// apply rc-mode (-> swap sticks)
		math::convertRcMode(parameters, man.y, man.x, man.r, man.z);

		// re-map throttle from [-1,1] -> [0,1] after swapping sticks
		man.z = math::gradual(man.z, -1.0f, 1.0f, 0.0f, 1.0f);

		// mode switch needs to be set to None such that slots are considered
		man.mode_switch = manual_control_setpoint_s::SWITCH_POS_NONE;
		man.kill_switch = updateKillSwitch(man); // process kill switch shortcut
		man.timestamp = input_rc.timestamp_last_signal;

		// reset state of previous _param_rc_map_aux
		rcmap_aux_changed = (int)_param_rcmap_aux.get() != _param_rcmap_aux_prev;
		_param_rcmap_aux_prev = _param_rcmap_aux.get();

		return (int)RCMap::Error::None;
	}

protected:
	// 12-Bit channel value limits with offset
	static constexpr int CALIBRATION_OFFSET = 95; // offset to make sure we reach all vlaues
	static constexpr int MIN_VALUE = 0 + CALIBRATION_OFFSET; // 0 + offset due to calibration
	static constexpr int MAX_VALUE = 4095 - CALIBRATION_OFFSET; // 12 bits - offset due to calibration

	// Channels (which have a common content)
	static constexpr int CHANNEL_LEFT_STICK_UP = (1 - 1);
	static constexpr int CHANNEL_LEFT_STICK_RIGHT = (2 - 1);
	static constexpr int CHANNEL_RIGHT_STICK_UP = (3 - 1);
	static constexpr int CHANNEL_RIGHT_STICK_RIGHT = (4 - 1);
	static constexpr int CHANNEL_THREE_WAY_SWITCH = (8 - 1);
	static constexpr int CHANNEL_TWO_WAY_SWITCH = (9 - 1);

	bool rcmap_aux_changed = false; //if purpose of _param_rc_map_aux has changed

	/**
	 * Convert 12-Bit M4 channel values to unit floats
	 * but cutting of the margin a bit (95) to make sure the extreme values are reachable
	 * @param value [0,4095] channel value
	 * @return [-1,1] floats
	 */
	float unit_range(uint16_t value)
	{
		return math::gradual<float>(value, MIN_VALUE, MAX_VALUE, -1.0f, 1.0f);
	};

	/**
	 * Convert two way switches
	 * from their channel bits [0,1] to manual_control_setpoint SWITCH_POS
	 * @param sw bit offset of the switch inside the channel data [0,1]
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	int two_way_switch(int offset_count, const int channel, const input_rc_s &input_rc)
	{
		if ((input_rc.values[channel] >> offset_count) & 0x1) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}
	}

	/**
	 * Convert three way switches
	 * from their channel bits [0,1,2] to manual_control_setpoint SWITCH_POS [2,1,0]
	 * @param sw bit offset of the switch inside the channel data
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	int three_way_switch(int offset_count, const int channel, const input_rc_s &input_rc)
	{
		switch ((input_rc.values[channel] >> (offset_count * 2)) & 0x3) {
		case 0: // switch is up
			return manual_control_setpoint_s::SWITCH_POS_OFF;

		case 1: // switch is middle
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		case 2: // switch is down
			return manual_control_setpoint_s::SWITCH_POS_ON;

		default:
			return manual_control_setpoint_s::SWITCH_POS_NONE;
		}
	}

	/**
	 * Convert an unlocked button into a locked button that keeps its state.
	 * @param offset_count bit offset of the channel data
	 * @param input_rc RC channel data
	 * return state of button
	 */
	int button(int offset_count, const input_rc_s &input_rc)
	{
		if ((input_rc.values[CHANNEL_TWO_WAY_SWITCH] >> offset_count) & 0x1) {
			_aux_button.update_state(true);

		} else {
			_aux_button.update_state(false);
		}

		if (_aux_button.get_state()) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}
	}

	/**
	 * Reset button to its initial state
	 */
	void reset_button()
	{
		_aux_button.reset_button();
	}

	/**
	 * Process Kill switch shortcut
	 * to trigger: press the arm button three times with low throttle within KILL_HOTKEY_TIME_US
	 * @param sw bit offset of the switch inside the channel data [0,1]
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	bool updateKillSwitch(const manual_control_setpoint_s &man)
	{
		const bool first_time = _kill_hotkey_count == 0;
		const bool hotkey_complete = _kill_hotkey_count >= KILL_SWITCH_TRIGGER_COUNT;
		const bool within_timeout = hrt_elapsed_time(&_kill_hotkey_start_time) < KILL_HOTKEY_TIME_US;

		if (hotkey_complete) {
			_kill_state = true;
		}

		if (man.z < 0.25f && (first_time || within_timeout) && !hotkey_complete) {
			if (!_arm_button_pressed_last && man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				if (first_time) {
					_kill_hotkey_start_time = hrt_absolute_time();
				}

				_kill_hotkey_count++;
			}

		}  else {
			// reset
			_kill_hotkey_count = 0;
			_kill_hotkey_start_time = 0;
		}

		_arm_button_pressed_last = man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON;
		return _kill_state;
	}

private:
	// Kill switch shortcut logic states
	static constexpr int KILL_HOTKEY_TIME_US = 1_s; // 1s time for kill-switch criteria
	static constexpr int KILL_SWITCH_TRIGGER_COUNT = 3; // arm button pushed three times -> kill
	bool _kill_state = false; // the kill state in which we lockdown the motors until restart
	hrt_abstime _kill_hotkey_start_time = 0; // the time when the hotkey started to measure timeout
	int _kill_hotkey_count = 0; //  how many times the button was pressed during the hotkey timeout
	bool _arm_button_pressed_last = false; //if the button was pressed last time to detect a transition
	Button _aux_button{false}; //aux button pressed / not pressed
	int _param_rcmap_aux_prev = 0; //previous state of _param_rc_map_aux
};
} // namespace sensors
