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

#include "OpticalFlowAccumulator.hpp"
#include "OpticalFlowGyroFeed.hpp"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/vehicle_optical_flow.h>

namespace sensors
{

/**
 * Optical flow on a CAN sensor node: pairs each frame with the node's gyro and
 * aggregates frames to a fixed rate for the bus.
 *
 * Rotation, scale, limits and timing are vehicle configuration and belong to
 * the flight controller, so this class has no parameters.
 */
class NodeOpticalFlow : public px4::ScheduledWorkItem
{
public:
	NodeOpticalFlow();
	~NodeOpticalFlow() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	static constexpr uint32_t kPublishIntervalUs = 1'000'000 / CONFIG_SENSORS_NODE_OPTICAL_FLOW_RATE;

	uORB::Publication<vehicle_optical_flow_s> _vehicle_optical_flow_pub{ORB_ID(vehicle_optical_flow)};

	uORB::SubscriptionCallbackWorkItem _sensor_flow_sub{this, ORB_ID(sensor_optical_flow)};

	OpticalFlowAccumulator _accumulator{};
	OpticalFlowGyroFeed _gyro_feed{this};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};

} // namespace sensors
