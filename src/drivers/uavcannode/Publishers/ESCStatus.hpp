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

#include <uavcan/equipment/esc/Status.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>

namespace uavcannode
{

class ESCStatus :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::esc::Status>
{
public:
	ESCStatus(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::esc::Status::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(esc_status)),
		uavcan::Publisher<uavcan::equipment::esc::Status>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::esc::Status::getDataTypeFullName(),
			       uavcan::equipment::esc::Status::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// esc_status -> uavcan::equipment::esc::Status
		esc_status_s esc_status;

		if (uORB::SubscriptionCallbackWorkItem::update(&esc_status)) {

			for (size_t i = 0; i < esc_status.esc_count; i++) {
				uavcan::equipment::esc::Status status{};

				status.esc_index = i;
				status.error_count = esc_status.esc[i].esc_errorcount;
				status.voltage = esc_status.esc[i].esc_voltage;
				status.current = esc_status.esc[i].esc_current;
				status.temperature = esc_status.esc[i].esc_temperature;
				status.rpm = esc_status.esc[i].esc_rpm;
				//status.power_rating_pct = NAN;

				uavcan::Publisher<uavcan::equipment::esc::Status>::broadcast(status);

				// ensure callback is registered
				uORB::SubscriptionCallbackWorkItem::registerCallback();
			}
		}
	}
};
} // namespace uavcan
