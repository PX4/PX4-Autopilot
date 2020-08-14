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
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "rangefinder.hpp"

const char *const UavcanRangefinderBridge::NAME = "range_finder";

#define UAVCAN_RANGEFINDER_BASE_DEVICE_PATH "/dev/uavcan/range_finder"
#define SF_LW20_C_MIN_DISTANCE 0.01
#define SF_LW20_C_MAX_DISTANCE 100.0

UavcanRangefinderBridge::UavcanRangefinderBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_rangefinder", "/dev/uavcan/range_finder", RANGE_FINDER_BASE_DEVICE_PATH, ORB_ID(distance_sensor)),
	_sub_measurement_data(node),
	_px4_rangefinder(DRV_DIST_DEVTYPE_SF1XX, ORB_PRIO_DEFAULT)
{}

int UavcanRangefinderBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_measurement_data.start(RangefinderCbBinder(this, &UavcanRangefinderBridge::measurement_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan rangefinder sub: %d", res);
		return res;
	}

	return 0;
}


void
UavcanRangefinderBridge::measurement_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg)
{
	_px4_rangefinder.set_min_distance(SF_LW20_C_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(SF_LW20_C_MAX_DISTANCE);

	_px4_rangefinder.update(hrt_absolute_time(), msg.range);
}
