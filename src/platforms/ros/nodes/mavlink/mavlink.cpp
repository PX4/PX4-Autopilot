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
	_v_att_sp_pub(_n.advertise<vehicle_attitude_setpoint>("vehicle_attitude_setpoint", 1)),
	_pos_sp_triplet_pub(_n.advertise<position_setpoint_triplet>("position_setpoint_triplet", 1)),
	_offboard_control_mode_pub(_n.advertise<offboard_control_mode>("offboard_control_mode", 1)),
	_force_sp_pub(_n.advertise<vehicle_force_setpoint>("vehicle_force_setpoint", 1))
{
	_link = mavconn::MAVConnInterface::open_url("udp://localhost:14565@localhost:14560");
	_link->message_received.connect(boost::bind(&Mavlink::handle_msg, this, _1, _2, _3));
	_att_sp = {};
	_offboard_control_mode = {};
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
		case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
			handle_msg_set_position_target_local_ned(mmsg);
			break;
		default:
			break;
	}

}

void Mavlink::handle_msg_set_attitude_target(const mavlink_message_t *mmsg)
{
	mavlink_set_attitude_target_t set_attitude_target;
	mavlink_msg_set_attitude_target_decode(mmsg, &set_attitude_target);

	/* set correct ignore flags for thrust field: copy from mavlink message */
	_offboard_control_mode.ignore_thrust = (bool)(set_attitude_target.type_mask & (1 << 6));

	/*
	 * if attitude or body rate have been used (not ignored) previously and this message only sends
	 * throttle and has the ignore bits set for attitude and rates don't change the flags for attitude and
	 * body rates to keep the controllers running
	 */
	bool ignore_bodyrate = (bool)(set_attitude_target.type_mask & 0x7);
	bool ignore_attitude = (bool)(set_attitude_target.type_mask & (1 << 7));

	if (ignore_bodyrate && ignore_attitude && !_offboard_control_mode.ignore_thrust) {
		/* Message want's us to ignore everything except thrust: only ignore if previously ignored */
		_offboard_control_mode.ignore_bodyrate = ignore_bodyrate && _offboard_control_mode.ignore_bodyrate;
		_offboard_control_mode.ignore_attitude = ignore_attitude && _offboard_control_mode.ignore_attitude;
	} else {
		_offboard_control_mode.ignore_bodyrate = ignore_bodyrate;
		_offboard_control_mode.ignore_attitude = ignore_attitude;
	}
	_offboard_control_mode.ignore_position = true;
	_offboard_control_mode.ignore_velocity = true;
	_offboard_control_mode.ignore_acceleration_force = true;

	_offboard_control_mode.timestamp = get_time_micros();
	_offboard_control_mode_pub.publish(_offboard_control_mode);

	/* The real mavlink app has a ckeck at this location which makes sure that the attitude setpoint
	 * gets published only if in offboard mode. We leave that out for now.
	 */

	_att_sp.timestamp = get_time_micros();
	if (!ignore_attitude) {
		mavlink_quaternion_to_euler(set_attitude_target.q, &_att_sp.roll_body, &_att_sp.pitch_body,
				&_att_sp.yaw_body);
		mavlink_quaternion_to_dcm(set_attitude_target.q, (float(*)[3])_att_sp.R_body.data());
		_att_sp.R_valid = true;
	}
	

	if (!_offboard_control_mode.ignore_thrust) {
		_att_sp.thrust = set_attitude_target.thrust;
	}

	if (!ignore_attitude) {
		for (ssize_t i = 0; i < 4; i++) {
			_att_sp.q_d[i] = set_attitude_target.q[i];
		}
		_att_sp.q_d_valid = true;
	}

	_v_att_sp_pub.publish(_att_sp);


	//XXX real mavlink publishes rate sp here

}

void Mavlink::handle_msg_set_position_target_local_ned(const mavlink_message_t *mmsg)
{

	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(mmsg, &set_position_target_local_ned);

	offboard_control_mode offboard_control_mode;
	// memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));//XXX breaks compatibility with multiple setpoints

	/* Only accept messages which are intended for this system */
	// XXX removed for sitl, makes maybe sense to re-introduce at some point
	// if ((mavlink_system.sysid == set_position_target_local_ned.target_system ||
				// set_position_target_local_ned.target_system == 0) &&
			// (mavlink_system.compid == set_position_target_local_ned.target_component ||
			 // set_position_target_local_ned.target_component == 0)) {

	/* convert mavlink type (local, NED) to uORB offboard control struct */
	offboard_control_mode.ignore_position = (bool)(set_position_target_local_ned.type_mask & 0x7);
	offboard_control_mode.ignore_velocity = (bool)(set_position_target_local_ned.type_mask & 0x38);
	offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_local_ned.type_mask & 0x1C0);
	bool is_force_sp = (bool)(set_position_target_local_ned.type_mask & (1 << 9));
	/* yaw ignore flag mapps to ignore_attitude */
	offboard_control_mode.ignore_attitude = (bool)(set_position_target_local_ned.type_mask & 0x400);
	/* yawrate ignore flag mapps to ignore_bodyrate */
	offboard_control_mode.ignore_bodyrate = (bool)(set_position_target_local_ned.type_mask & 0x800);



	offboard_control_mode.timestamp = get_time_micros();
	_offboard_control_mode_pub.publish(offboard_control_mode);

	/* The real mavlink app has a ckeck at this location which makes sure that the position setpoint triplet
	 * gets published only if in offboard mode. We leave that out for now.
	 */
	if (is_force_sp && offboard_control_mode.ignore_position &&
			offboard_control_mode.ignore_velocity) {
		/* The offboard setpoint is a force setpoint only, directly writing to the force
		 * setpoint topic and not publishing the setpoint triplet topic */
		vehicle_force_setpoint	force_sp;
		force_sp.x = set_position_target_local_ned.afx;
		force_sp.y = set_position_target_local_ned.afy;
		force_sp.z = set_position_target_local_ned.afz;
		//XXX: yaw

		_force_sp_pub.publish(force_sp);
	} else {
		/* It's not a pure force setpoint: publish to setpoint triplet  topic */
		position_setpoint_triplet pos_sp_triplet;
		pos_sp_triplet.previous.valid = false;
		pos_sp_triplet.next.valid = false;
		pos_sp_triplet.current.valid = true;
		pos_sp_triplet.current.type = position_setpoint::SETPOINT_TYPE_POSITION; //XXX support others

		/* set the local pos values */
		if (!offboard_control_mode.ignore_position) {
			pos_sp_triplet.current.position_valid = true;
			pos_sp_triplet.current.x = set_position_target_local_ned.x;
			pos_sp_triplet.current.y = set_position_target_local_ned.y;
			pos_sp_triplet.current.z = set_position_target_local_ned.z;
		} else {
			pos_sp_triplet.current.position_valid = false;
		}

		/* set the local vel values */
		if (!offboard_control_mode.ignore_velocity) {
			pos_sp_triplet.current.velocity_valid = true;
			pos_sp_triplet.current.vx = set_position_target_local_ned.vx;
			pos_sp_triplet.current.vy = set_position_target_local_ned.vy;
			pos_sp_triplet.current.vz = set_position_target_local_ned.vz;
		} else {
			pos_sp_triplet.current.velocity_valid = false;
		}

		/* set the local acceleration values if the setpoint type is 'local pos' and none
		 * of the accelerations fields is set to 'ignore' */
		if (!offboard_control_mode.ignore_acceleration_force) {
			pos_sp_triplet.current.acceleration_valid = true;
			pos_sp_triplet.current.a_x = set_position_target_local_ned.afx;
			pos_sp_triplet.current.a_y = set_position_target_local_ned.afy;
			pos_sp_triplet.current.a_z = set_position_target_local_ned.afz;
			pos_sp_triplet.current.acceleration_is_force =
				is_force_sp;

		} else {
			pos_sp_triplet.current.acceleration_valid = false;
		}

		/* set the yaw sp value */
		if (!offboard_control_mode.ignore_attitude && !isnan(set_position_target_local_ned.yaw)) {
			pos_sp_triplet.current.yaw_valid = true;
			pos_sp_triplet.current.yaw = set_position_target_local_ned.yaw;

		} else {
			pos_sp_triplet.current.yaw_valid = false;
		}

		/* set the yawrate sp value */
		if (!offboard_control_mode.ignore_bodyrate && !isnan(set_position_target_local_ned.yaw)) {
			pos_sp_triplet.current.yawspeed_valid = true;
			pos_sp_triplet.current.yawspeed = set_position_target_local_ned.yaw_rate;

		} else {
			pos_sp_triplet.current.yawspeed_valid = false;
		}
		//XXX handle global pos setpoints (different MAV frames)

		_pos_sp_triplet_pub.publish(pos_sp_triplet);
	}
}
