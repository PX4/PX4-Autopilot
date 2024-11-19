/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed.h>
#include <uavcan/equipment/air_data/IndicatedAirspeed.hpp>

namespace uavcannode
{

class IndicatedAirspeed : public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher <
	uavcan::equipment::air_data::IndicatedAirspeed >
{
public:
	IndicatedAirspeed(px4::WorkItem *work_item, uavcan::INode &node)
		: UavcanPublisherBase(
			  uavcan::equipment::air_data::IndicatedAirspeed::DefaultDataTypeID),
		  uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(airspeed)),
		  uavcan::Publisher<uavcan::equipment::air_data::IndicatedAirspeed>(
			  node)
	{
		this->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf(
				"\t%s -> %s:%d\n",
				uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
				uavcan::equipment::air_data::IndicatedAirspeed::getDataTypeFullName(),
				uavcan::equipment::air_data::IndicatedAirspeed::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{

		// airspeed -> uavcan::equipment::air_data::IndicatedAirspeed

		airspeed_s airspeed_m;

		if (uORB::SubscriptionCallbackWorkItem::update(&airspeed_m)) {
			uavcan::equipment::air_data::IndicatedAirspeed indicated_as{};

			indicated_as.indicated_airspeed = airspeed_m.indicated_airspeed_m_s;
			uavcan::Publisher<uavcan::equipment::air_data::IndicatedAirspeed>::
			broadcast(indicated_as);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
