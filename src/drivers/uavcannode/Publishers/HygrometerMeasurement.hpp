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

#include <dronecan/sensors/hygrometer/Hygrometer.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_hygrometer.h>

namespace uavcannode
{

class HygrometerMeasurement :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<dronecan::sensors::hygrometer::Hygrometer>
{
public:
	HygrometerMeasurement(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(dronecan::sensors::hygrometer::Hygrometer::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_hygrometer)),
		uavcan::Publisher<dronecan::sensors::hygrometer::Hygrometer>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       dronecan::sensors::hygrometer::Hygrometer::getDataTypeFullName(),
			       dronecan::sensors::hygrometer::Hygrometer::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// sensor_hygrometer -> dronecan::sensors::hyrometer::Hygrometer
		sensor_hygrometer_s hygro;

		if ((hrt_elapsed_time(&_last_hygrometer_publish) > 1_s) && uORB::SubscriptionCallbackWorkItem::update(&hygro)) {

			// hygrometer temp in degrees C, humidity in percent
			dronecan::sensors::hygrometer::Hygrometer hygro_measurement{};

			hygro_measurement.temperature = hygro.temperature;
			hygro_measurement.humidity = hygro.humidity;

			uavcan::Publisher<dronecan::sensors::hygrometer::Hygrometer>::broadcast(hygro_measurement);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();

			_last_hygrometer_publish = hrt_absolute_time();
		}
	}
private:
	hrt_abstime _last_hygrometer_publish{0};
};
} // namespace uavcannode
