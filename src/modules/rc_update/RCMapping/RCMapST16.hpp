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
 * @file ST16.hpp
 *
 * Defining ST16 channel parsing to map input data to manual_control_setpoint.
 *
 * @author Dennis Mannhart <dennis@yuneecresearch.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMapST.hpp"

namespace sensors
{

class RCMapST16 : public RCMapST
{
public:
	RCMapST16() = default;
	virtual ~RCMapST16() = default;

	static constexpr int RAW_CHANNEL_MAPPING_VER_ST16 = 0xA; // Remote type and mapping version

	/**
	 * Map ST16 data (channels) to the manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	virtual int map(manual_control_setpoint_s &man, const input_rc_s &input_rc, int32_t &parameters)
	{
		// analog inputs
		man.aux1 = unit_range(input_rc.values[CHANNEL_PAN_KNOB]); // pan-knob / gimbal yaw
		man.aux2 = unit_range(input_rc.values[CHANNEL_TILT_SLIDER]); // tilt-slider / gimbal tilt
		man.aux5 = unit_range(input_rc.values[CHANNEL_TORTOISE_SLIDER]); // turtle-slider

		// switches
		man.mode_slot = mode_switch((int)ThreeWay::mode_switch, CHANNEL_THREE_WAY_SWITCH, input_rc);
		man.obsavoid_switch = three_way_switch((int)ThreeWay::obs_avoid_switch, CHANNEL_THREE_WAY_SWITCH, input_rc);
		man.gimbal_yaw_mode = three_way_switch((int)ThreeWay::pan_switch, CHANNEL_THREE_WAY_SWITCH, input_rc);
		man.gimbal_pitch_mode = three_way_switch((int)ThreeWay::tilt_switch, CHANNEL_THREE_WAY_SWITCH, input_rc);
		man.gear_switch = two_way_switch((int)TwoWay::gear_switch, CHANNEL_TWO_WAY_SWITCH, input_rc);

		if (rcmap_aux_changed) {
			// reset button since purpose of button has changed
			reset_button();
			man.mission_button = manual_control_setpoint_s::SWITCH_POS_OFF;
			man.flexi_release = manual_control_setpoint_s::SWITCH_POS_OFF;
		}

		if (_param_rcmap_aux.get() == (int)RCMap::AUX::mission) {
			// aux is set to mission/loiter
			man.mission_button = button((int)TwoWay::aux_button, input_rc);

		} else if (_param_rcmap_aux.get() == (int)RCMap::AUX::flexi_release) {
			man.flexi_release = two_way_switch((int)TwoWay::aux_button, CHANNEL_TWO_WAY_SWITCH, input_rc);
		}

		if (!_param_rcmap_aux.get()) {
			// aux is not mapped: set everything to default
			man.mission_button = manual_control_setpoint_s::SWITCH_POS_OFF;

		} else if (_param_rcmap_aux.get() == (int)RCMap::AUX::mission) {
			// aux is set to mission/loiter
			man.mission_button = button((int)TwoWay::aux_button, input_rc);
		}

		// buttons
		man.arm_switch = two_way_switch((int)TwoWay::arm_button, CHANNEL_TWO_WAY_SWITCH, input_rc);

		// TODO: add remaining buttons: AUX, photo, video, trim

		return RCMapST::map(man, input_rc, parameters);
	}

	/**
	 * Map ST16 team mode slave data (channels) on top of the existing manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	int mapSlave(manual_control_setpoint_s &man, const input_rc_s &slave_rc)
	{
		// switches
		man.gimbal_yaw_mode = three_way_switch((int)ThreeWay::pan_switch, CHANNEL_THREE_WAY_SWITCH,
						       slave_rc); // OFF: heading 0, MNIDDLE: angle, ON: angle stabilized
		man.gimbal_pitch_mode = three_way_switch((int)ThreeWay::tilt_switch, CHANNEL_THREE_WAY_SWITCH,
					slave_rc); // OFF/MIDDLE: angle, ON: velocity

		// analog inputs
		man.aux1 = unit_range(slave_rc.values[CHANNEL_RIGHT_STICK_RIGHT]); // camera pan (= yaw)

		// camera tilt (= pitch)
		if (man.gimbal_pitch_mode == manual_control_setpoint_s::SWITCH_POS_ON) {
			man.aux2 = unit_range(slave_rc.values[CHANNEL_LEFT_STICK_UP]); // tilt speed control: use stick inputs

		} else {
			man.aux2 = unit_range(slave_rc.values[CHANNEL_TILT_SLIDER]); // tilt angle control: use tilt slider

		}

		// do not overwrite man.timestamp because master is the main required input that triggers rc loss!
		return (int)RCMap::Error::None;
	}

private:
	// Channels
	static constexpr int CHANNEL_PAN_KNOB = (5 - 1);
	static constexpr int CHANNEL_TILT_SLIDER = (6 - 1);
	static constexpr int CHANNEL_TORTOISE_SLIDER = (7 - 1);
	static constexpr int CHANNEL_TRIM = (10 - 1);

	enum class ThreeWay : int {
		mode_switch = 0,
		obs_avoid_switch,
		pan_switch,
		tilt_switch
	};

	enum class TwoWay : int {
		gear_switch = 0,
		arm_button,
		aux_button,
		photo_button,
		video_button,
	};

	enum class Trim : int {
		left_trim_up = 0,
		left_trim_down,
		left_trim_left,
		left_trim_right,
		right_trim_up,
		right_trim_down,
		right_trim_left,
		right_trim_right
	};

	/**
	 * Convert three way mode switch
	 * from their channel bits [0,1,2] to manual_control_setpoint MODE_SLOT
	 * @param sw bit offset of the switch inside the channel data
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	int mode_switch(int offset_count, const int channel, const input_rc_s &input_rc)
	{
		// mode slots define the mode of the vehicle.
		const int mode_switch = three_way_switch(offset_count, channel, input_rc);

		switch (mode_switch) {
		case manual_control_setpoint_s::SWITCH_POS_OFF: // switch is up - Altitude
			return manual_control_setpoint_s::MODE_SLOT_1;

		case manual_control_setpoint_s::SWITCH_POS_MIDDLE: // switch is middle - Position
			return manual_control_setpoint_s::MODE_SLOT_2;

		case manual_control_setpoint_s::SWITCH_POS_ON: // switch is down - Return
			return manual_control_setpoint_s::MODE_SLOT_3;

		default:
			return manual_control_setpoint_s::MODE_SLOT_NONE;
		}
	}
};

} // namespace sensors
