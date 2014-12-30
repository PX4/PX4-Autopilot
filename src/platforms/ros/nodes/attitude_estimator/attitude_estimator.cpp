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
 * Dummy attitude estimator that forwards attitude from gazebo to px4 topic
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "attitude_estimator.h"

#include <px4/vehicle_attitude.h>

AttitudeEstimator::AttitudeEstimator() :
	_n(),
	_sub_modelstates(_n.subscribe("joy", 10, &AttitudeEstimator::ModelStatesCallback, this)),
	_vehicle_attitude_pub(_n.advertise<px4::vehicle_attitude>("vehicle_attitude", 10))
{
}

void AttitudeEstimator::ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
	px4::vehicle_attitude msg_out;

	/* Fill px4 attitude topic with contents from modelstates topic */
	//XXX

	_vehicle_attitude_pub.publish(msg_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_estimator");
  AttitudeEstimator m;

  ros::spin();

  return 0;
}
