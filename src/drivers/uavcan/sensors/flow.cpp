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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "flow.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

const char *const UavcanFlowBridge::NAME = "flow";

UavcanFlowBridge::UavcanFlowBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_flow", "/dev/uavcan/flow", FLOW_BASE_DEVICE_PATH, ORB_ID(optical_flow)),
	_sub_flow(node)
{
	// _device_id.devid_s.devtype = DRV_FLOW_DEVTYPE;     // <-- Why?????????????????
}

int UavcanFlowBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_flow.start(FlowCbBinder(this, &UavcanFlowBridge::flow_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanFlowBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	// static uint64_t last_read = 0;
	// struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
  //
	// /* buffer must be large enough */
	// unsigned count = buflen / sizeof(struct mag_report);
  //
	// if (count < 1) {
	// 	return -ENOSPC;
	// }
  //
	// if (last_read < _report.timestamp) {
	// 	/* copy report */
	// 	lock();
	// 	*mag_buf = _report;
	// 	last_read = _report.timestamp;
	// 	unlock();
	// 	return sizeof(struct mag_report);
  //
	// } else {
	// 	/* no new data available, warn caller */
	// 	return -EAGAIN;
	// }
  return -EAGAIN;
}

int UavcanFlowBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	// switch (cmd) {
  //
	// case MAGIOCSSCALE: {
	// 		std::memcpy(&_scale, reinterpret_cast<const void *>(arg), sizeof(_scale));
	// 		return 0;
	// 	}
  //
	// case MAGIOCGSCALE: {
	// 		std::memcpy(reinterpret_cast<void *>(arg), &_scale, sizeof(_scale));
	// 		return 0;
	// 	}
  //
	// case MAGIOCGEXTERNAL: {
	// 		return 1;           // declare it external rise it's priority and to allow for correct orientation compensation
	// 	}
  //
	// case MAGIOCCALIBRATE:
	// case MAGIOCSRANGE:
	// case MAGIOCEXSTRAP: {
	// 		return -EINVAL;
	// 	}
  //
	// default: {
	// 		return CDev::ioctl(filp, cmd, arg);
	// 	}
	// }
  return -1;
}

void UavcanFlowBridge::flow_sub_cb(const
		uavcan::ReceivedDataStructure<com::hex::equipment::flow::Measurement>
		&msg)
{
	lock();
	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	_report.timestamp = hrt_absolute_time();
	_report.sensor_id = 0;

	_report.pixel_flow_x_integral = msg.flow_integral[0];
	_report.pixel_flow_y_integral = msg.flow_integral[1];

	_report.gyro_x_rate_integral = msg.rate_gyro_integral[0];
	_report.gyro_y_rate_integral = msg.rate_gyro_integral[1];

	_report.quality = msg.quality;
	_report.max_flow_rate = 5.0f;       // Datasheet: 7.4 rad/s
	_report.min_ground_distance = 0.1f; // Datasheet: 80mm
	_report.max_ground_distance = 30.0f; // Datasheet: infinity


	unlock();

	publish(msg.getSrcNodeID().get(), &_report);
}
