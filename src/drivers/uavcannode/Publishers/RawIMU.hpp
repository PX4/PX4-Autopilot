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

#include <uavcan/equipment/ahrs/RawIMU.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_imu.h>

namespace uavcannode
{

class RawIMU :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>
{
public:
	RawIMU(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::ahrs::RawIMU::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(vehicle_imu)),
		uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::ahrs::RawIMU::getDataTypeFullName(),
			       uavcan::equipment::ahrs::RawIMU::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// vehicle_imu -> uavcan::equipment::ahrs::RawIMU
		vehicle_imu_s vehicle_imu;

		if (uORB::SubscriptionCallbackWorkItem::update(&vehicle_imu)) {

			uavcan::equipment::ahrs::RawIMU raw_imu{};

			raw_imu.timestamp.usec = getNode().getUtcTime().toUSec() - (hrt_absolute_time() -
						 vehicle_imu.timestamp_sample);

			raw_imu.integration_interval = vehicle_imu.delta_angle_dt;
			// raw_imu.integration_interval = vehicle_imu.delta_velocity_dt;

			raw_imu.rate_gyro_latest[0] = (vehicle_imu.delta_angle[0] / vehicle_imu.delta_angle_dt) * 1000000;
			raw_imu.rate_gyro_latest[1] = (vehicle_imu.delta_angle[1] / vehicle_imu.delta_angle_dt) * 1000000;
			raw_imu.rate_gyro_latest[2] = (vehicle_imu.delta_angle[2] / vehicle_imu.delta_angle_dt) * 1000000;

			raw_imu.rate_gyro_integral[0] = vehicle_imu.delta_angle[0];
			raw_imu.rate_gyro_integral[1] = vehicle_imu.delta_angle[1];
			raw_imu.rate_gyro_integral[2] = vehicle_imu.delta_angle[2];

			raw_imu.accelerometer_latest[0] = (vehicle_imu.delta_velocity[0] / vehicle_imu.delta_velocity_dt) * 1000000;
			raw_imu.accelerometer_latest[1] = (vehicle_imu.delta_velocity[1] / vehicle_imu.delta_velocity_dt) * 1000000;
			raw_imu.accelerometer_latest[2] = (vehicle_imu.delta_velocity[2] / vehicle_imu.delta_velocity_dt) * 1000000;

			raw_imu.accelerometer_integral[0] = vehicle_imu.delta_velocity[0];
			raw_imu.accelerometer_integral[1] = vehicle_imu.delta_velocity[1];
			raw_imu.accelerometer_integral[2] = vehicle_imu.delta_velocity[2];

			uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>::broadcast(raw_imu);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
