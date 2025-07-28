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

#include <cmath>

#include "UavcanPublisherBase.hpp"

#include <uavcan/navigation/GlobalNavigationSolution.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_local_position.h>

namespace uavcannode
{

class GlobalNavigationSolution :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::navigation::GlobalNavigationSolution>
{
public:
	GlobalNavigationSolution(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::navigation::GlobalNavigationSolution::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(vehicle_local_position)),
		uavcan::Publisher<uavcan::navigation::GlobalNavigationSolution>(node)
	{
		this->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::navigation::GlobalNavigationSolution::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::navigation::GlobalNavigationSolution;

		// vehicle_local_position -> uavcan::navigation::GlobalNavigationSolution
		vehicle_local_position_s localposition;

		if (uORB::SubscriptionCallbackWorkItem::update(&localposition)) {
			uavcan::navigation::GlobalNavigationSolution navigationsolution{};

			//navigationsolution.timestamp = 0; // get uavcan monotonic time?

			if (localposition.xy_global) {
				navigationsolution.latitude  = localposition.ref_lat;
				navigationsolution.longitude = localposition.ref_lon;
			}

			if (localposition.z_global) {
				//navigationsolution.height_ellipsoid = localposition.alt_ellipsoid;
				navigationsolution.height_msl       = localposition.ref_alt;
			}

			//navigationsolution.height_agl
			//navigationsolution.height_baro

			//navigationsolution.qnh_hpa

			//navigationsolution.orientation_xyzw

			//navigationsolution.pose_covariance

			navigationsolution.linear_velocity_body[0] = localposition.vx;
			navigationsolution.linear_velocity_body[1] = localposition.vy;
			navigationsolution.linear_velocity_body[2] = localposition.vz;

			//navigationsolution.angular_velocity_body

			navigationsolution.linear_acceleration_body[0] = localposition.ax;
			navigationsolution.linear_acceleration_body[1] = localposition.ay;
			navigationsolution.linear_acceleration_body[2] = localposition.az;

			//navigationsolution.velocity_covariance

			uavcan::Publisher<uavcan::navigation::GlobalNavigationSolution>::broadcast(navigationsolution);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
