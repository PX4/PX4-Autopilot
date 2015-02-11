/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink.cpp
 * Dummy mavlink node that interfaces to a mavros node via UDP
 * This simulates the onboard mavlink app to some degree. It should be possible to
 * send offboard setpoints via mavros to the SITL setup the same way as on the real system
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "mavlink.h"

#include <platforms/px4_middleware.h>

using namespace px4;

Mavlink::Mavlink() :
	_n(),
	_v_att_sub(_n.subscribe("vehicle_attitude", 10, &Mavlink::VehicleAttitudeCallback, this))
{

	_link = mavconn::MAVConnInterface::open_url("udp://localhost:14565@localhost:14560");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavlink");
	Mavlink m;
	ros::spin();
	return 0;
}

void Mavlink::VehicleAttitudeCallback(const px4::vehicle_attitudeConstPtr &msg)
{
	mavlink_message_t msg_m;
	mavlink_msg_attitude_quaternion_pack_chan(
			_link->get_system_id(),
			_link->get_component_id(),
			_link->get_channel(),
			&msg_m, //XXX hardcoded
			get_time_micros() / 1000,
			msg->q[0],
			msg->q[1],
			msg->q[2],
			msg->q[3],
			msg->rollspeed,
			msg->pitchspeed,
			msg->yawspeed);
	_link->send_message(&msg_m);
}
