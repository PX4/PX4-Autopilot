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
 * @file ST10C.hpp
 *
 * Defining ST10C channel parsing to map input data to manual_control_setpoint.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMapST.hpp"

namespace sensors
{

class RCMapST10C : public RCMapST
{
public:
	RCMapST10C() = default;
	virtual ~RCMapST10C() = default;

	static constexpr int RAW_CHANNEL_MAPPING_VER_ST10C = 0xC; // Remote type and mapping version

	/**
	 * Map ST10C data (channels) to the manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	virtual int map(manual_control_setpoint_s &man, const input_rc_s &input_rc, int32_t &parameters)
	{
		// analog inputs
		man.aux1 = 0; // pan-knob / gimbal yaw (not present so we set it to neutral)
		man.aux2 = unit_range(input_rc.values[CHANNEL_LEFT_WHEEL]); // tilt-slider / gimbal tilt
		man.aux5 = 1; // turtle-slider (not present so we set the value to 100%)

		// switches
		man.gimbal_yaw_mode = manual_control_setpoint_s::SWITCH_POS_OFF; // Pan is forward only (pan knob is unused)
		man.gimbal_pitch_mode = manual_control_setpoint_s::SWITCH_POS_ON; // Tilt fixed in velocity mode

		// buttons
		man.arm_switch = two_way_switch((int)TwoWay::arm_button, CHANNEL_TWO_WAY_SWITCH, input_rc);
		man.loiter_switch = two_way_switch((int)TwoWay::loiter_button, CHANNEL_TWO_WAY_SWITCH, input_rc);
		man.return_switch = two_way_switch((int)TwoWay::rtl_button, CHANNEL_TWO_WAY_SWITCH, input_rc);
		// Hardcode mode to be in POSCTL.
		man.mode_slot = manual_control_setpoint_s::MODE_SLOT_2;
		//const bool photo_button_pressed = two_way_switch(TwoWay::photo_button, CHANNEL_TWO_WAY_SWITCH, input_rc);
		//const bool video_button_pressed = two_way_switch(TwoWay::video_button, CHANNEL_TWO_WAY_SWITCH, input_rc);

		man.gear_switch = three_way_switch((int)ThreeWay::right_switch, CHANNEL_THREE_WAY_SWITCH, input_rc);

		return RCMapST::map(man, input_rc, parameters);
	}

	/**
	 * Map ST10C team mode slave data (channels) on top of the existing manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	int mapSlave(manual_control_setpoint_s &man, const input_rc_s &slave_rc)
	{
		// currently unsupported
		return (int)Error::Version;
	}

private:
	// Channels
	static constexpr int CHANNEL_LEFT_WHEEL = (5 - 1);
	static constexpr int CHANNEL_RIGHT_WHEEL = (6 - 1);
	static constexpr int CHANNEL_EMPTY = (7 - 1);

	enum class ThreeWay : int {
		right_switch = 0,
	};

	enum class TwoWay : int {
		arm_button = 0,
		loiter_button,
		rtl_button,
		photo_button,
		video_button,
	};

};

} // namespace sensors
