/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author RJ Gritter <rjgritter657@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include "rangefinder.hpp"
#include <math.h>

const char *const UavcanRangefinderBridge::NAME = "rangefinder";

UavcanRangefinderBridge::UavcanRangefinderBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_rangefinder", "/dev/uavcan/rangefinder", RANGE_FINDER_BASE_DEVICE_PATH, ORB_ID(distance_sensor)),
	_sub_range_data(node)
{ }

int UavcanRangefinderBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_range_data.start(RangeCbBinder(this, &UavcanRangefinderBridge::range_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanRangefinderBridge::range_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg)
{
	distance_sensor_s report{};

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	report.timestamp   		= hrt_absolute_time();
	report.current_distance   	= msg.range;
	report.h_fov			= msg.field_of_view;
	report.v_fov			= msg.field_of_view;

	// report.min_distance =

	report.variance   		= 0.0f;	// Unknown
	report.signal_quality		= -1;

	switch (msg.sensor_type) {
		case 0:	// Undefined - Assume laser
			report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		break;
		case 1:	// Sonar
			report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		break;
		case 2:	// Lidar
			report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		break;
		case 3:	// Radar
			report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
		break;
	}

	publish(msg.getSrcNodeID().get(), &report);
}
