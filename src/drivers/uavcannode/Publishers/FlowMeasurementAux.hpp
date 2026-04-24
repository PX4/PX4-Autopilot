/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <com/ark/equipment/flow/MeasurementAux.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_optical_flow.h>

namespace uavcannode
{

class FlowMeasurementAux :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<com::ark::equipment::flow::MeasurementAux>
{
public:
	FlowMeasurementAux(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(com::ark::equipment::flow::MeasurementAux::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(vehicle_optical_flow)),
		uavcan::Publisher<com::ark::equipment::flow::MeasurementAux>(node)
	{
		this->setPriority(uavcan::TransferPriority::Default);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       com::ark::equipment::flow::MeasurementAux::getDataTypeFullName(),
			       com::ark::equipment::flow::MeasurementAux::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		vehicle_optical_flow_s optical_flow;

		if (uORB::SubscriptionCallbackWorkItem::update(&optical_flow)) {
			com::ark::equipment::flow::MeasurementAux measurement{};

			measurement.shutter = optical_flow.shutter;

			// DSDL illumination_mode: 0=bright, 1=low, 2=super-low, 3=unknown.
			// uORB mode:              0=unknown, 1=bright, 2=low, 3=super-low.
			switch (optical_flow.mode) {
			case vehicle_optical_flow_s::MODE_BRIGHT:         measurement.illumination_mode = 0; break;
			case vehicle_optical_flow_s::MODE_LOWLIGHT:       measurement.illumination_mode = 1; break;
			case vehicle_optical_flow_s::MODE_SUPER_LOWLIGHT: measurement.illumination_mode = 2; break;
			default:                                          measurement.illumination_mode = 3; break;
			}

			measurement.motion              = optical_flow.motion;
			measurement.challenging_surface = optical_flow.challenging_surface;
			measurement.chip_health_ok      = optical_flow.chip_health_ok;
			measurement.discard_count       = optical_flow.discard_count;
			measurement.mode_change_count   = optical_flow.mode_change_count;

			uavcan::Publisher<com::ark::equipment::flow::MeasurementAux>::broadcast(measurement);

			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
