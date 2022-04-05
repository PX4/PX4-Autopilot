/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "UavcanPublisherBase.hpp"

#include <ardupilot/gnss/RelPosHeading.hpp>

#include <lib/drivers/device/Device.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_gnss_relative.h>

namespace uavcannode
{

class RelPosHeadingPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ardupilot::gnss::RelPosHeading>
{
public:
	RelPosHeadingPub(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::gnss::RelPosHeading::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_gnss_relative)),
		uavcan::Publisher<ardupilot::gnss::RelPosHeading>(node)
	{
		this->setPriority(uavcan::TransferPriority::NumericallyMax);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::gnss::RelPosHeading::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using ardupilot::gnss::RelPosHeading;

		// sensor_gnss_relative -> ardupilot::gnss::RelPosHeading
		sensor_gnss_relative_s sensor_gnss_relative;

		if (uORB::SubscriptionCallbackWorkItem::update(&sensor_gnss_relative)) {
			ardupilot::gnss::RelPosHeading rel_pos_heading{};

			// TODO: FIX (timestamp_sample and UAVCAN timestamp)
			rel_pos_heading.timestamp = uavcan::UtcTime::fromUSec(getNode().getMonotonicTime().toUSec());

			rel_pos_heading.reported_heading_acc_available = sensor_gnss_relative.heading_valid; // bool
			rel_pos_heading.reported_heading_deg = math::degrees(sensor_gnss_relative.heading); // float32
			rel_pos_heading.reported_heading_acc_deg = math::degrees(sensor_gnss_relative.heading_accuracy); // float32
			rel_pos_heading.relative_distance_m = sensor_gnss_relative.position_length; // float16
			rel_pos_heading.relative_down_pos_m = sensor_gnss_relative.position[2]; // float16

			uavcan::Publisher<ardupilot::gnss::RelPosHeading>::broadcast(rel_pos_heading);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
