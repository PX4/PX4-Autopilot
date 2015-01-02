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
 * @file commander.cpp
 * Dummy commander node that publishes the various status topics
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "commander.h"

#include <px4/manual_control_setpoint.h>
#include <px4/vehicle_control_mode.h>
#include <px4/vehicle_status.h>
#include <platforms/px4_middleware.h>

Commander::Commander() :
	_n(),
	_man_ctrl_sp_sub(_n.subscribe("manual_control_setpoint", 10, &Commander::ManualControlInputCallback, this)),
	_vehicle_control_mode_pub(_n.advertise<px4::vehicle_control_mode>("vehicle_control_mode", 10)),
	_actuator_armed_pub(_n.advertise<px4::actuator_armed>("actuator_armed", 10)),
	_vehicle_status_pub(_n.advertise<px4::vehicle_status>("vehicle_status", 10)),
	_parameter_update_pub(_n.advertise<px4::parameter_update>("parameter_update", 10)),
	_msg_parameter_update(),
	_msg_actuator_armed()
{
}

void Commander::ManualControlInputCallback(const px4::manual_control_setpointConstPtr& msg)
{
	px4::vehicle_control_mode msg_vehicle_control_mode;
	px4::vehicle_status msg_vehicle_status;

	/* fill vehicle control mode */
	//XXX hardcoded
	msg_vehicle_control_mode.timestamp = px4::get_time_micros();
	msg_vehicle_control_mode.flag_control_manual_enabled = true;
	msg_vehicle_control_mode.flag_control_rates_enabled = true;
	msg_vehicle_control_mode.flag_control_attitude_enabled = true;

	/* fill actuator armed */
	float arm_th = 0.95;
	_msg_actuator_armed.timestamp = px4::get_time_micros();
	if (_msg_actuator_armed.armed) {
		/* Check for disarm */
		if (msg->r < -arm_th && msg->z < (1-arm_th)) {
			_msg_actuator_armed.armed = false;
		}
	} else {
		/* Check for arm */
		if (msg->r > arm_th && msg->z < (1-arm_th)) {
			_msg_actuator_armed.armed = true;
		}
	}

	/* fill vehicle status */
	//XXX hardcoded
	msg_vehicle_status.timestamp = px4::get_time_micros();
	msg_vehicle_status.main_state = msg_vehicle_status.MAIN_STATE_MANUAL;
	msg_vehicle_status.nav_state = msg_vehicle_status.NAVIGATION_STATE_MANUAL;
	msg_vehicle_status.arming_state = msg_vehicle_status.ARMING_STATE_ARMED;
	msg_vehicle_status.hil_state = msg_vehicle_status.HIL_STATE_OFF;
	msg_vehicle_status.hil_state = msg_vehicle_status.VEHICLE_TYPE_QUADROTOR;
	msg_vehicle_status.is_rotary_wing = true;

	_vehicle_control_mode_pub.publish(msg_vehicle_control_mode);
	_actuator_armed_pub.publish(_msg_actuator_armed);
	_vehicle_status_pub.publish(msg_vehicle_status);

	/* Fill parameter update */
	if (px4::get_time_micros() - _msg_parameter_update.timestamp > 1e6) {
		_msg_parameter_update.timestamp = px4::get_time_micros();
		_parameter_update_pub.publish(_msg_parameter_update);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");
  Commander m;
  ros::spin();
  return 0;
}
