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
 * @file manual_input.h
 * Reads from the ros joystick topic and publishes to the px4 manual control setpoint topic.
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

class ManualInput {
public:
	ManualInput();

	~ManualInput() {}

protected:
	/**
	 * Takes ROS joystick message and converts/publishes to px4 manual control setpoint topic
	 */
	void JoyCallback(const sensor_msgs::JoyConstPtr& msg);

	/**
	 * Helper function to map and scale joystick input
	 */
	void MapAxis(const sensor_msgs::JoyConstPtr& msg, int map_index, double scale, double offset, double deadzone,
			float &out);

	ros::NodeHandle _n;
	ros::Subscriber _joy_sub;
	ros::Publisher _man_ctrl_sp_pub;

	/* Parameters */
	int _param_axes_map[4];
	double _param_axes_scale[4];
	double _param_axes_offset[4];
	double _param_axes_dz[4];


};
