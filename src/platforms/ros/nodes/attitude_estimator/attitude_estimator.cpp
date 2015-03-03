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
 * @file att_estimator.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Roman Bapst <romanbapst@yahoo.de>
*/

#include "attitude_estimator.h"

#include <px4/vehicle_attitude.h>
#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>
#include <platforms/px4_middleware.h>

AttitudeEstimator::AttitudeEstimator() :
	_n(),
	// _sub_modelstates(_n.subscribe("/gazebo/model_states", 1, &AttitudeEstimator::ModelStatesCallback, this)),
	_vehicle_attitude_pub(_n.advertise<px4::vehicle_attitude>("vehicle_attitude", 1))
{
	std::string vehicle_model;
	_n.param("vehicle_model", vehicle_model, std::string("iris"));
	_sub_imu = _n.subscribe("/" + vehicle_model + "/imu", 1, &AttitudeEstimator::ImuCallback, this);
}

void AttitudeEstimator::ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
	px4::vehicle_attitude msg_v_att;

	/* Fill px4 attitude topic with contents from modelstates topic */

	/* Convert quaternion to rotation matrix */
	math::Quaternion quat;
	//XXX: search for ardrone or other (other than 'plane') vehicle here
	int index = 1;
	quat(0) = (float)msg->pose[index].orientation.w;
	quat(1) = (float)msg->pose[index].orientation.x;
	quat(2) = (float) - msg->pose[index].orientation.y;
	quat(3) = (float) - msg->pose[index].orientation.z;

	msg_v_att.q[0] = quat(0);
	msg_v_att.q[1] = quat(1);
	msg_v_att.q[2] = quat(2);
	msg_v_att.q[3] = quat(3);

	math::Matrix<3, 3> rot = quat.to_dcm();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			PX4_R(msg_v_att.R, i, j) = rot(i, j);
		}
	}

	msg_v_att.R_valid = true;

	math::Vector<3> euler = rot.to_euler();
	msg_v_att.roll = euler(0);
	msg_v_att.pitch = euler(1);
	msg_v_att.yaw = euler(2);

	//XXX this is in inertial frame
	// msg_v_att.rollspeed = (float)msg->twist[index].angular.x;
	// msg_v_att.pitchspeed = -(float)msg->twist[index].angular.y;
	// msg_v_att.yawspeed = -(float)msg->twist[index].angular.z;

	msg_v_att.timestamp = px4::get_time_micros();
	_vehicle_attitude_pub.publish(msg_v_att);
}

void AttitudeEstimator::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
	px4::vehicle_attitude msg_v_att;

	/* Fill px4 attitude topic with contents from modelstates topic */

	/* Convert quaternion to rotation matrix */
	math::Quaternion quat;
	//XXX: search for ardrone or other (other than 'plane') vehicle here
	int index = 1;
	quat(0) = (float)msg->orientation.w;
	quat(1) = (float)msg->orientation.x;
	quat(2) = (float) - msg->orientation.y;
	quat(3) = (float) - msg->orientation.z;

	msg_v_att.q[0] = quat(0);
	msg_v_att.q[1] = quat(1);
	msg_v_att.q[2] = quat(2);
	msg_v_att.q[3] = quat(3);

	math::Matrix<3, 3> rot = quat.to_dcm();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			PX4_R(msg_v_att.R, i, j) = rot(i, j);
		}
	}

	msg_v_att.R_valid = true;

	math::Vector<3> euler = rot.to_euler();
	msg_v_att.roll = euler(0);
	msg_v_att.pitch = euler(1);
	msg_v_att.yaw = euler(2);

	msg_v_att.rollspeed = (float)msg->angular_velocity.x;
	msg_v_att.pitchspeed = -(float)msg->angular_velocity.y;
	msg_v_att.yawspeed = -(float)msg->angular_velocity.z;

	msg_v_att.timestamp = px4::get_time_micros();
	_vehicle_attitude_pub.publish(msg_v_att);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "attitude_estimator");
	AttitudeEstimator m;

	ros::spin();

	return 0;
}
