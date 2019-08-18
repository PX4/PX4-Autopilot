/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "rangefinder.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanRangefinderBridge::NAME = "rangefinder";

UavcanRangefinderBridge::UavcanRangefinderBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_rangefinder", "/dev/uavcan/rangefinder", "/dev/rangefinder", ORB_ID(distance_sensor)),
	_sub_rangefinder(node)
{
}

int
UavcanRangefinderBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_rangefinder.start(RangefinderCbBinder(this, &UavcanRangefinderBridge::rangefinder_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanRangefinderBridge::rangefinder_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg)
{
	distance_sensor_s rangefinder{};

	// float roll, pitch, yaw = 0;

	rangefinder.timestamp = hrt_absolute_time();
	rangefinder.current_distance = msg.range;

	if (msg.reading_type == msg.READING_TYPE_UNDEFINED) {
		rangefinder.signal_quality = -1;
	} else if (msg.reading_type == msg.READING_TYPE_VALID_RANGE) {
		rangefinder.signal_quality = 100;
	} else {
		rangefinder.signal_quality = 0;
	}

	// how do we convert these to PX4 rotations
	// if (msg.beam_orientation_in_body_frame.orientation_defined) {
	// 	roll = msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0] *
	// 		msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;

	// 	pitch = msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] *
	// 		msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;

	// 	yaw = msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2] *
	// 		msg.beam_orientation_in_body_frame.ANGLE_MULTIPLIER;
	// }

	// since currently the only sensor using it will be the HEX optical flow,
	// we can keep the downwards rotations for now
	rangefinder.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	switch(msg.sensor_type) {
		case msg.SENSOR_TYPE_UNDEFINED : // this case is not good !!!
			rangefinder.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
			break;
		case msg.SENSOR_TYPE_SONAR :
			rangefinder.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
			break;
		case msg.SENSOR_TYPE_LIDAR :
			rangefinder.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
			break;
		case msg.SENSOR_TYPE_RADAR :
			rangefinder.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
			break;
	}

	rangefinder.id = msg.sensor_id;
	rangefinder.h_fov = msg.field_of_view;

	publish(msg.getSrcNodeID().get(), &rangefinder);
}
