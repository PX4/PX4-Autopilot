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
	_v_att_sub(_n.subscribe("vehicle_attitude", 1, &Mavlink::VehicleAttitudeCallback, this)),
	_v_local_pos_sub(_n.subscribe("vehicle_local_position", 1, &Mavlink::VehicleLocalPositionCallback, this)),
	_offboard_control_mode_pub(_n.advertise<offboard_control_mode>("offboard_control_mode", 1))
{
	_link = mavconn::MAVConnInterface::open_url("udp://localhost:14565@localhost:14560");
	_link->message_received.connect(boost::bind(&Mavlink::handle_msg, this, _1, _2, _3));

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavlink");
	Mavlink m;
	ros::spin();
	return 0;
}

void Mavlink::VehicleAttitudeCallback(const vehicle_attitudeConstPtr &msg)
{
	mavlink_message_t msg_m;
	mavlink_msg_attitude_quaternion_pack_chan(
			_link->get_system_id(),
			_link->get_component_id(),
			_link->get_channel(),
			&msg_m,
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

void Mavlink::VehicleLocalPositionCallback(const vehicle_local_positionConstPtr &msg)
{
	mavlink_message_t msg_m;
	mavlink_msg_local_position_ned_pack_chan(
			_link->get_system_id(),
			_link->get_component_id(),
			_link->get_channel(),
			&msg_m,
			get_time_micros() / 1000,
			msg->x,
			msg->y,
			msg->z,
			msg->vx,
			msg->vy,
			msg->vz);
	_link->send_message(&msg_m);
}

void Mavlink::handle_msg(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
	(void)sysid;
	(void)compid;

	switch(mmsg->msgid) {
		case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
			handle_msg_set_attitude_target(mmsg);
			break;
		default:
			break;
	}

}

void Mavlink::handle_msg_set_attitude_target(const mavlink_message_t *mmsg)
{
	mavlink_set_attitude_target_t set_att_target;
	mavlink_msg_set_attitude_target_decode(mmsg, &set_att_target);

	offboard_control_mode offboard_control_mode_msg;
	/* set correct ignore flags for body rate fields: copy from mavlink message */
	offboard_control_mode_msg.ignore_bodyrate = (bool)(set_att_target.type_mask & 0x7);
	/* set correct ignore flags for thrust field: copy from mavlink message */
	offboard_control_mode_msg.ignore_thrust = (bool)(set_att_target.type_mask & (1 << 6));
	/* set correct ignore flags for attitude field: copy from mavlink message */
	offboard_control_mode_msg.ignore_attitude = (bool)(set_att_target.type_mask & (1 << 7));

	offboard_control_mode_msg.timestamp = get_time_micros();
	_offboard_control_mode_pub.publish(offboard_control_mode_msg);

	vehicle_attitude_setpoint v_att_sp_msg;

	/* The real mavlink app has a ckeck at this location which makes sure that the attitude setpoint
	 * gets published only if in offboard mode. We leave that out for now.
	 * */

	v_att_sp_msg.timestamp = get_time_micros();
	mavlink_quaternion_to_euler(set_att_target.q, &v_att_sp_msg.roll_body, &v_att_sp_msg.pitch_body,
			&v_att_sp_msg.yaw_body);
	mavlink_quaternion_to_dcm(set_att_target.q, (float(*)[3])v_att_sp_msg.R_body.data());
	v_att_sp_msg.R_valid = true;
	v_att_sp_msg.thrust = set_att_target.thrust;
	for (ssize_t i = 0; i < 4; i++) {
		v_att_sp_msg.q_d[i] = set_att_target.q[i];
	}
	v_att_sp_msg.q_d_valid = true;

	_v_att_sp_pub.publish(v_att_sp_msg);

}
