/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file manual_input.cpp
 * Reads from the ros joystick topic and publishes to the px4 manual control input topic.
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "manual_input.h"

#include <platforms/px4_middleware.h>

ManualInput::ManualInput() :
	_n(),
	_joy_sub(_n.subscribe("joy", 1, &ManualInput::JoyCallback, this)),
	_man_ctrl_sp_pub(_n.advertise<px4::manual_control_setpoint>("manual_control_setpoint", 1)),
	_buttons_state(),
	_msg_mc_sp()
{
	double dz_default = 0.2;
	/* Load parameters, default values work for Microsoft XBox Controller */
	_n.param("map_x", _param_axes_map[0], 4);
	_n.param("scale_x", _param_axes_scale[0], 1.0);
	_n.param("off_x", _param_axes_offset[0], 0.0);
	_n.param("dz_x", _param_axes_dz[0], dz_default);

	_n.param("map_y", _param_axes_map[1], 3);
	_n.param("scale_y", _param_axes_scale[1], -1.0);
	_n.param("off_y", _param_axes_offset[1], 0.0);
	_n.param("dz_y", _param_axes_dz[1], dz_default);

	_n.param("map_z", _param_axes_map[2], 2);
	_n.param("scale_z", _param_axes_scale[2], -0.5);
	_n.param("off_z", _param_axes_offset[2], -1.0);
	_n.param("dz_z", _param_axes_dz[2], 0.0);

	_n.param("map_r", _param_axes_map[3], 0);
	_n.param("scale_r", _param_axes_scale[3], -1.0);
	_n.param("off_r", _param_axes_offset[3], 0.0);
	_n.param("dz_r", _param_axes_dz[3], dz_default);

	_n.param("map_manual", _param_buttons_map[0], 0);
	_n.param("map_altctl", _param_buttons_map[1], 1);
	_n.param("map_posctl", _param_buttons_map[2], 2);
	_n.param("map_auto_mission", _param_buttons_map[3], 3);
	_n.param("map_auto_loiter", _param_buttons_map[4], 4);
	_n.param("map_auto_rtl", _param_buttons_map[5], 5);
	_n.param("map_offboard", _param_buttons_map[6], 6);

	/* Default to manual */
	_msg_mc_sp.mode_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.posctl_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.loiter_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.offboard_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
	_msg_mc_sp.acro_switch = px4::manual_control_setpoint::SWITCH_POS_NONE;

}

void ManualInput::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{

	/* Fill px4 manual control setpoint topic with contents from ros joystick */
	/* Map sticks to x, y, z, r */
	MapAxis(msg, _param_axes_map[0], _param_axes_scale[0], _param_axes_offset[0], _param_axes_dz[0], _msg_mc_sp.x);
	MapAxis(msg, _param_axes_map[1], _param_axes_scale[1], _param_axes_offset[1], _param_axes_dz[1], _msg_mc_sp.y);
	MapAxis(msg, _param_axes_map[2], _param_axes_scale[2], _param_axes_offset[2], _param_axes_dz[2], _msg_mc_sp.z);
	MapAxis(msg, _param_axes_map[3], _param_axes_scale[3], _param_axes_offset[3], _param_axes_dz[3], _msg_mc_sp.r);
	//ROS_INFO("x: %1.4f y: %1.4f z: %1.4f r: %1.4f", msg_mc_sp.x, msg_out.y, msg_out.z, msg_out.r);

	/* Map buttons to switches */
	MapButtons(msg, _msg_mc_sp);

	_msg_mc_sp.timestamp = px4::get_time_micros();

	_man_ctrl_sp_pub.publish(_msg_mc_sp);
}

void ManualInput::MapAxis(const sensor_msgs::JoyConstPtr &msg, int map_index, double scale, double offset,
			  double deadzone, float &out)
{
	double val = msg->axes[map_index];

	if (val + offset > deadzone || val + offset < -deadzone) {
		out = (float)((val + offset)) * scale;

	} else {
		out = 0.0f;
	}
}

void ManualInput::MapButtons(const sensor_msgs::JoyConstPtr &msg, px4::manual_control_setpoint &msg_mc_sp)
{
	msg_mc_sp.acro_switch = px4::manual_control_setpoint::SWITCH_POS_NONE;

	if (_buttons_state[MAIN_STATE_MANUAL] != msg->buttons[_param_buttons_map[MAIN_STATE_MANUAL]] == true) {
		msg_mc_sp.mode_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.posctl_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.loiter_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.offboard_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		return;

	} else if (_buttons_state[MAIN_STATE_ALTCTL] != msg->buttons[_param_buttons_map[MAIN_STATE_ALTCTL]] == true) {
		msg_mc_sp.mode_switch = px4::manual_control_setpoint::SWITCH_POS_MIDDLE;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.posctl_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.loiter_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.offboard_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		return;

	} else if (_buttons_state[MAIN_STATE_POSCTL] != msg->buttons[_param_buttons_map[MAIN_STATE_POSCTL]] == true) {
		msg_mc_sp.mode_switch = px4::manual_control_setpoint::SWITCH_POS_MIDDLE;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.posctl_switch = px4::manual_control_setpoint::SWITCH_POS_ON;
		msg_mc_sp.loiter_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.offboard_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		return;

	} else if (_buttons_state[MAIN_STATE_OFFBOARD] != msg->buttons[_param_buttons_map[MAIN_STATE_OFFBOARD]] == true) {
		msg_mc_sp.mode_switch = px4::manual_control_setpoint::SWITCH_POS_MIDDLE;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.posctl_switch = px4::manual_control_setpoint::SWITCH_POS_ON;
		msg_mc_sp.loiter_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.return_switch = px4::manual_control_setpoint::SWITCH_POS_OFF;
		msg_mc_sp.offboard_switch = px4::manual_control_setpoint::SWITCH_POS_ON;
		return;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "manual_input");
	ManualInput m;
	ros::spin();
	return 0;
}
