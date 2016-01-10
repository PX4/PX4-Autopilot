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
 * @file commander.h
 * Dummy commander node that publishes the various status topics
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "ros/ros.h"
#include <px4/manual_control_setpoint.h>
#include <px4/vehicle_control_mode.h>
#include <px4/vehicle_status.h>
#include <px4/parameter_update.h>
#include <px4/actuator_armed.h>
#include <px4/offboard_control_mode.h>

class Commander
{
public:
	Commander();

	~Commander() {}

protected:
	/**
	 * Based on manual control input the status will be set
	 */
	void ManualControlInputCallback(const px4::manual_control_setpointConstPtr &msg);

	/**
	 * Stores the offboard control mode
	 */
	void OffboardControlModeCallback(const px4::offboard_control_modeConstPtr &msg);

	/**
	 * Set control mode flags based on stick positions (equiv to code in px4 commander)
	 */
	void EvalSwitches(const px4::manual_control_setpointConstPtr &msg,
			  px4::vehicle_status &msg_vehicle_status,
			  px4::vehicle_control_mode &msg_vehicle_control_mode);

	/**
	 * Sets offboard control flags in msg_vehicle_control_mode
	 */
	void SetOffboardControl(const px4::offboard_control_mode &msg_offboard_control_mode,
				px4::vehicle_control_mode &msg_vehicle_control_mode);

	ros::NodeHandle _n;
	ros::Subscriber _man_ctrl_sp_sub;
	ros::Subscriber _offboard_control_mode_sub;
	ros::Publisher _vehicle_control_mode_pub;
	ros::Publisher _actuator_armed_pub;
	ros::Publisher _vehicle_status_pub;
	ros::Publisher _parameter_update_pub;

	px4::parameter_update _msg_parameter_update;
	px4::actuator_armed _msg_actuator_armed;
	px4::vehicle_control_mode _msg_vehicle_control_mode;
	px4::vehicle_status _msg_vehicle_status;
	px4::offboard_control_mode _msg_offboard_control_mode;

	bool _got_manual_control;

};
