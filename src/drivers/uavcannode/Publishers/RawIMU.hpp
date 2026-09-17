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

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
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

			// timestamp is the acquisition time of the latest sample, which is also the
			// end of the integration window: VehicleIMU stamps vehicle_imu with the
			// sensor sample that closed the window.
			raw_imu.timestamp.usec = bus_timestamp_usec(getNode(), vehicle_imu.timestamp_sample);

			// integration_interval is in seconds; delta_angle_dt is in microseconds
			raw_imu.integration_interval = vehicle_imu.delta_angle_dt * 1e-6f;

			// *_latest is the latest point sample, not the window mean: the mean lags
			// the timestamp by half the interval, and consumers that want the mean
			// have the integral. The sensor topics are the driver's uncalibrated
			// output, as a point sample from any other IMU driver would be.
			sensor_gyro_s gyro;

			if (latest_sample(_sensor_gyro_sub, vehicle_imu.gyro_device_id, gyro)) {
				raw_imu.rate_gyro_latest[0] = gyro.x;
				raw_imu.rate_gyro_latest[1] = gyro.y;
				raw_imu.rate_gyro_latest[2] = gyro.z;

			} else {
				raw_imu.rate_gyro_latest[0] = (vehicle_imu.delta_angle[0] / vehicle_imu.delta_angle_dt) * 1000000;
				raw_imu.rate_gyro_latest[1] = (vehicle_imu.delta_angle[1] / vehicle_imu.delta_angle_dt) * 1000000;
				raw_imu.rate_gyro_latest[2] = (vehicle_imu.delta_angle[2] / vehicle_imu.delta_angle_dt) * 1000000;
			}

			raw_imu.rate_gyro_integral[0] = vehicle_imu.delta_angle[0];
			raw_imu.rate_gyro_integral[1] = vehicle_imu.delta_angle[1];
			raw_imu.rate_gyro_integral[2] = vehicle_imu.delta_angle[2];

			sensor_accel_s accel;

			if (latest_sample(_sensor_accel_sub, vehicle_imu.accel_device_id, accel)) {
				raw_imu.accelerometer_latest[0] = accel.x;
				raw_imu.accelerometer_latest[1] = accel.y;
				raw_imu.accelerometer_latest[2] = accel.z;

			} else {
				raw_imu.accelerometer_latest[0] = (vehicle_imu.delta_velocity[0] / vehicle_imu.delta_velocity_dt) * 1000000;
				raw_imu.accelerometer_latest[1] = (vehicle_imu.delta_velocity[1] / vehicle_imu.delta_velocity_dt) * 1000000;
				raw_imu.accelerometer_latest[2] = (vehicle_imu.delta_velocity[2] / vehicle_imu.delta_velocity_dt) * 1000000;
			}

			raw_imu.accelerometer_integral[0] = vehicle_imu.delta_velocity[0];
			raw_imu.accelerometer_integral[1] = vehicle_imu.delta_velocity[1];
			raw_imu.accelerometer_integral[2] = vehicle_imu.delta_velocity[2];

			uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>::broadcast(raw_imu);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}

private:
	// Copy the newest sample of the sensor instance feeding vehicle_imu, re-selecting
	// the instance if the one subscribed does not carry the expected device id.
	template<typename SampleT>
	static bool latest_sample(uORB::Subscription &sub, uint32_t device_id, SampleT &sample)
	{
		if (sub.copy(&sample) && sample.device_id == device_id) {
			return true;
		}

		for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
			uORB::Subscription candidate{sub.orb_id(), instance};

			if (candidate.copy(&sample) && sample.device_id == device_id) {
				sub.ChangeInstance(instance);
				return true;
			}
		}

		return false;
	}

	uORB::Subscription _sensor_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
};
} // namespace uavcannode
