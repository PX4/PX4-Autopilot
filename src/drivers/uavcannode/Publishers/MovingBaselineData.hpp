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
#include <drivers/drv_hrt.h>
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

		// Drain all available messages from the queue and forward to CAN.
		// The GPS driver publishes complete RTCM messages as back-to-back fragments,
		// so forwarding them in order preserves message integrity.
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

				if (result >= 0) {
					_msg_count++;
					_msg_count_per_interval++;
					_bytes_count += chunk_size;
				}

				mbd.data.clear();
			}
		}

		// Print stats every 5 seconds
		hrt_abstime now = hrt_absolute_time();

		if (now > _last_stats_time + 5000000ULL) {
			float dt = (now - _last_stats_time) / 1e6f;
			PX4_INFO("MBD TX: %u msgs (%.1f/s), %u bytes",
				 (unsigned)_msg_count,
				 (double)(_msg_count_per_interval / dt),
				 (unsigned)_bytes_count);
			_last_stats_time = now;
			_msg_count_per_interval = 0;
		}

		// Store the generation for next time
		_last_generation = uORB::SubscriptionCallbackWorkItem::get_last_generation();

		// ensure callback is registered
		uORB::SubscriptionCallbackWorkItem::registerCallback();
	}

private:
	unsigned _last_generation{0};

	uint32_t _msg_count{0};
	uint32_t _msg_count_per_interval{0};
	uint32_t _bytes_count{0};
	hrt_abstime _last_stats_time{0};
};
} // namespace uavcannode
