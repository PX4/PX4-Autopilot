/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <ardupilot/gnss/MovingBaselineData.hpp>

#include <lib/drivers/device/Device.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/gps_inject_data.h>

namespace uavcannode
{

class MovingBaselineDataPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ardupilot::gnss::MovingBaselineData>
{
public:
	MovingBaselineDataPub(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::gnss::MovingBaselineData::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(gps_inject_data)),
		uavcan::Publisher<ardupilot::gnss::MovingBaselineData>(node)
	{
		this->setPriority(uavcan::TransferPriority::NumericallyMax);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::gnss::MovingBaselineData::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		// Check for dropped messages before we start reading
		if (_last_generation != 0) {
			unsigned current_generation = uORB::SubscriptionCallbackWorkItem::get_last_generation();
			unsigned expected_generation = _last_generation + 1;

			if (current_generation > expected_generation) {
				unsigned messages_lost = current_generation - expected_generation;
				PX4_ERR("MovingBaselineData: dropped %u messages (queue overflow)", messages_lost);
			}
		}

		// gps_inject_data -> ardupilot::gnss::MovingBaselineData
		gps_inject_data_s gps_inject_data;

		// TODO: The GpsInjectData uORB topic is published multiple times from the gps.cpp driver such that the whole RTCM message
		// received from the module arrives at the Node publisher back-to-back. So basically we want
		// to take that uorb data and assemble it into as few MovingBaselineData messages as possible. We also want to make sure the
		// entire message is published onto the CAN bus without delay. Perhaps our CAN bus message emitter (this code right here)
		// should parse the gps_inject_data into a buffer and emit bursts of MovingBaselineData messages which contain the full
		// RTCM message. This way every burst of data onto the CAN bus is an atomic RTCM message. In the case that we get corrupted
		// data, our parser should be able to detect/recover from this without overflowing its buffer. So yeah, we basically just
		// need a parser that properly decodes (detects the framing, doesn't need a full decode) RTCM messages so that we can be sure
		// we emit the data fragments for the messages back to back.

		// Drain all available messages from the queue
		while (uORB::SubscriptionCallbackWorkItem::update(&gps_inject_data)) {
			// Prevent republishing rtcm data we received from uavcan
			union device::Device::DeviceId device_id;
			device_id.devid = gps_inject_data.device_id;

			if (device_id.devid_s.bus_type == device::Device::DeviceBusType::DeviceBusType_UAVCAN) {
				continue;
			}

			ardupilot::gnss::MovingBaselineData mbd = {};

			const size_t capacity = mbd.data.capacity();
			size_t written = 0;
			int result = 0;

			while ((result >= 0) && written < gps_inject_data.len) {
				size_t chunk_size = gps_inject_data.len - written;

				if (chunk_size > capacity) {
					chunk_size = capacity;
				}

				for (size_t i = 0; i < chunk_size; i++) {
					mbd.data.push_back(gps_inject_data.data[written]);
					written += 1;
				}

				result = uavcan::Publisher<ardupilot::gnss::MovingBaselineData>::broadcast(mbd);

				mbd.data.clear();
			}
		}

		// Store the generation for next time
		_last_generation = uORB::SubscriptionCallbackWorkItem::get_last_generation();

		// ensure callback is registered
		uORB::SubscriptionCallbackWorkItem::registerCallback();
	}

private:
	unsigned _last_generation{0};
};
} // namespace uavcannode
