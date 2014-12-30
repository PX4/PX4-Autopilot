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

#include <px4/manual_control_setpoint.h>

ManualInput::ManualInput() :
	_n(),
	_sub_joy(_n.subscribe("joy", 10, &ManualInput::JoyCallback, this)),
	_man_ctrl_sp_pub(_n.advertise<px4::manual_control_setpoint>("manual_control_setpoint", 1000))
{
}


void ManualInput::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
	px4::manual_control_setpoint msg_out;

	/* Fill px4 manual control setpoint topic with contents from ros joystick */
	//XXX

	_man_ctrl_sp_pub.publish(msg_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_input");
  ManualInput m;

  ros::spin();

  return 0;
}
