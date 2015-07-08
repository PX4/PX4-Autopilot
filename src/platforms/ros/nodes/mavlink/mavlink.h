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
 * @file mavlink.h
 * Dummy mavlink node that interfaces to a mavros node via UDP
 * This simulates the onboard mavlink app to some degree. It should be possible to
 * send offboard setpoints via mavros to the SITL setup the same way as on the real system
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
*/

#include "ros/ros.h"
#include <mavconn/interface.h>
#include <px4/vehicle_attitude.h>
#include <px4/vehicle_local_position.h>
#include <px4/vehicle_attitude_setpoint.h>
#include <px4/position_setpoint_triplet.h>
#include <px4/vehicle_force_setpoint.h>
#include <px4/offboard_control_mode.h>

namespace px4
{

class Mavlink
{
public:
	Mavlink();

	~Mavlink() {}

protected:

	ros::NodeHandle _n;
	mavconn::MAVConnInterface::Ptr _link;
	ros::Subscriber _v_att_sub;
	ros::Subscriber _v_local_pos_sub;
	ros::Publisher _v_att_sp_pub;
	ros::Publisher _pos_sp_triplet_pub;
	ros::Publisher _offboard_control_mode_pub;
	ros::Publisher _force_sp_pub;
	vehicle_attitude_setpoint _att_sp;
	offboard_control_mode _offboard_control_mode;

	/**
	 *
	 * Simulates output of attitude data from the FCU
	 * Equivalent to the mavlink stream ATTITUDE_QUATERNION
	 *
	 * */
	void VehicleAttitudeCallback(const vehicle_attitudeConstPtr &msg);

	/**
	 *
	 * Simulates output of local position data from the FCU
	 * Equivalent to the mavlink stream LOCAL_POSITION_NED
	 *
	 * */
	void VehicleLocalPositionCallback(const vehicle_local_positionConstPtr &msg);


	/**
	 *
	 * Handle incoming mavlink messages ant publish them to ROS ("Mavlink Receiver")
	 *
	 * */
	void handle_msg(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid);

	/**
	 *
	 * Handle SET_ATTITUDE_TARGET mavlink messages
	 *
	 * */
	void handle_msg_set_attitude_target(const mavlink_message_t *mmsg);

	/**
	 *
	 * Handle SET_POSITION_TARGET_LOCAL_NED mavlink messages
	 *
	 * */
	void handle_msg_set_position_target_local_ned(const mavlink_message_t *mmsg);

};

}
