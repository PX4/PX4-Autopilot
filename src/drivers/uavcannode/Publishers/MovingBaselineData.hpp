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
#include <lib/gnss/rtcm.h>
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

		// Feed all incoming gps_inject_data chunks into the RTCM3 frame parser.
		// The parser buffers data and detects RTCM message boundaries, allowing us to
		// emit complete RTCM messages as atomic bursts onto the CAN bus.
		while (uORB::SubscriptionCallbackWorkItem::update(&gps_inject_data)) {
			// Prevent republishing rtcm data we received from uavcan
			union device::Device::DeviceId device_id;
			device_id.devid = gps_inject_data.device_id;

			if (device_id.devid_s.bus_type == device::Device::DeviceBusType::DeviceBusType_UAVCAN) {
				continue;
			}

			size_t added = _rtcm_parser.addData(gps_inject_data.data, gps_inject_data.len);

			if (added < gps_inject_data.len) {
				PX4_WARN("MovingBaselineData: parser buffer full, %zu bytes dropped",
					 gps_inject_data.len - added);
			}
		}

		uint8_t rtcm_frame[gnss::RTCM3_MAX_FRAME_LEN] = {};
		size_t rtcm_len = {};

		while (_rtcm_parser.getNextMessage(rtcm_frame, &rtcm_len)) {
			broadcastRtcmMessage(rtcm_frame, rtcm_len);
		}

		// Store the generation for next time
		_last_generation = uORB::SubscriptionCallbackWorkItem::get_last_generation();

		// ensure callback is registered
		uORB::SubscriptionCallbackWorkItem::registerCallback();
	}

private:
	/**
	 * Broadcast a complete RTCM message as a burst of MovingBaselineData CAN frames.
	 * This ensures the entire RTCM message is sent atomically without interleaving
	 * with other messages.
	 */
	void broadcastRtcmMessage(const uint8_t *data, size_t len)
	{
		ardupilot::gnss::MovingBaselineData mbd = {};
		const size_t capacity = mbd.data.capacity();
		size_t written = 0;
		int result = 0;

		while ((result >= 0) && written < len) {
			size_t chunk_size = len - written;

			if (chunk_size > capacity) {
				chunk_size = capacity;
			}

			for (size_t i = 0; i < chunk_size; i++) {
				mbd.data.push_back(data[written]);
				written += 1;
			}

			result = uavcan::Publisher<ardupilot::gnss::MovingBaselineData>::broadcast(mbd);

			mbd.data.clear();
		}
	}

	unsigned _last_generation{0};
	gnss::Rtcm3Parser _rtcm_parser;
};
} // namespace uavcannode
