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

#include "gnss_relative.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/Limits.hpp>

const char *const UavcanGnssRelativeBridge::NAME = "gnss_relative";

UavcanGnssRelativeBridge::UavcanGnssRelativeBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_gnss_relative", ORB_ID(sensor_gnss_relative), node_info_publisher),
	_sub_rel_pos_heading(node)
{
	set_device_type(DRV_GNSS_DEVTYPE_UAVCAN);
}

int
UavcanGnssRelativeBridge::init()
{
	int res = _sub_rel_pos_heading.start(RelPosHeadingCbBinder(this, &UavcanGnssRelativeBridge::rel_pos_heading_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanGnssRelativeBridge::rel_pos_heading_sub_cb(const
		uavcan::ReceivedDataStructure<ardupilot::gnss::RelPosHeading> &msg)
{
	sensor_gnss_relative_s sensor_gnss_relative{};

	// A node timestamp that can't be used falls back to the receive time, which carries none of the receiver latency:
	// timestamp_sample is left at 0 so that SENS_GNSSn_DELAY applies.
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timestamp_sample = uavcan_bridge::sample_timestamp(msg.timestamp.usec,
					     _sub_rel_pos_heading.getNode().getUtcTime().toUSec(), now);
	sensor_gnss_relative.timestamp_sample = (timestamp_sample != now) ? timestamp_sample : 0;

	sensor_gnss_relative.heading_valid = msg.reported_heading_acc_available;
	sensor_gnss_relative.heading = math::radians(msg.reported_heading_deg);
	sensor_gnss_relative.heading_accuracy = math::radians(msg.reported_heading_acc_deg);
	sensor_gnss_relative.position_length = msg.relative_distance_m;
	sensor_gnss_relative.position[2] = msg.relative_down_pos_m;

	sensor_gnss_relative.device_id = make_uavcan_device_id(msg);

	// Register GNSS capability with NodeInfoPublisher after first successful message
	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				sensor_gnss_relative.device_id,
				NodeInfoPublisher::DeviceCapability::GPS);
	}

	sensor_gnss_relative.timestamp = hrt_absolute_time();

	// The heading fails with the receiver. Its sensor_gnss instance is looked up only while a failure is injected.
	_failure_config.update();
	const int8_t channel = get_channel_index_for_node(msg.getSrcNodeID().get());

	if (channel >= 0) {
		failure_injection::Mode mode = failure_injection::Mode::Ok;

		if (_failure_config.any_active()) {
			const int instance = gnss_instance(sensor_gnss_relative.device_id);

			if (instance >= 0) {
				mode = _failure_config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, instance + 1);
			}
		}

		if (!failure_injection::process(mode, sensor_gnss_relative, _stuck[channel])) {
			return;
		}
	}

	publish(msg.getSrcNodeID().get(), &sensor_gnss_relative);
}

int UavcanGnssRelativeBridge::gnss_instance(uint32_t device_id)
{
	for (int i = 0; i < _sensor_gnss_sub.size(); i++) {
		sensor_gnss_s sensor_gnss;

		if (_sensor_gnss_sub[i].copy(&sensor_gnss) && (sensor_gnss.device_id == device_id)) {
			return i;
		}
	}

	return -1;
}
