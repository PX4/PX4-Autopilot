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
 * @file position_estimator.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Roman Bapst <romanbapst@yahoo.de>
*/

#include "position_estimator.h"

#include <px4/vehicle_local_position.h>
#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>
#include <platforms/px4_middleware.h>
#include <vector>
#include <string>
#include <gazebo_msgs/ModelStates.h>

PositionEstimator::PositionEstimator() :
	_n(),
	_sub_modelstates(_n.subscribe("/gazebo/model_states", 1, &PositionEstimator::ModelStatesCallback, this)),
	_vehicle_position_pub(_n.advertise<px4::vehicle_local_position>("vehicle_local_position", 1)),
	_startup_time(1)
{
	_n.getParam("vehicle_model", _model_name);
}

void PositionEstimator::ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
	//XXX: use a proper sensor topic

	px4::vehicle_local_position msg_v_l_pos;

	/* Fill px4 position topic with contents from modelstates topic */
	int index = 0;

	//XXX: maybe a more clever approach would be to do this not on every loop, need to check if and when
	//gazebo rearranges indexes.
	for (std::vector<std::string>::const_iterator it = msg->name.begin(); it != msg->name.end(); ++it) {
		if (*it ==  _model_name) {
			index = it -  msg->name.begin();
			break;
		}
	}

	msg_v_l_pos.xy_valid = true;
	msg_v_l_pos.z_valid = true;
	msg_v_l_pos.v_xy_valid = true;
	msg_v_l_pos.v_z_valid = true;

	msg_v_l_pos.x = msg->pose[index].position.x;
	msg_v_l_pos.y = -msg->pose[index].position.y;
	msg_v_l_pos.z = -msg->pose[index].position.z;
	msg_v_l_pos.vx = msg->twist[index].linear.x;
	msg_v_l_pos.vy = -msg->twist[index].linear.y;
	msg_v_l_pos.vz = -msg->twist[index].linear.z;

	msg_v_l_pos.xy_global = true;
	msg_v_l_pos.z_global = true;
	msg_v_l_pos.ref_timestamp = _startup_time;
	msg_v_l_pos.ref_lat = 47.378301;
	msg_v_l_pos.ref_lon = 8.538777;
	msg_v_l_pos.ref_alt = 1200.0f;


	msg_v_l_pos.timestamp = px4::get_time_micros();
	_vehicle_position_pub.publish(msg_v_l_pos);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_estimator");
	PositionEstimator m;

	ros::spin();

	return 0;
}
