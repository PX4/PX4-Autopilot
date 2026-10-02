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


#include "NodeOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace time_literals;

NodeOpticalFlow::NodeOpticalFlow() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_optical_flow_pub.advertise();
}

NodeOpticalFlow::~NodeOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool NodeOpticalFlow::Start()
{
	_sensor_flow_sub.registerCallback();

	_gyro_feed.start();

	ScheduleNow();
	return true;
}

void NodeOpticalFlow::Stop()
{
	Deinit();

	_sensor_flow_sub.unregisterCallback();
	_gyro_feed.stop();
}

void NodeOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	if (_accumulator.gyroRequired()) {
		_gyro_feed.update(_accumulator);
	}

	sensor_optical_flow_s sensor_optical_flow;

	if (_sensor_flow_sub.update(&sensor_optical_flow)) {

		_accumulator.addFrame(sensor_optical_flow);

		if (_accumulator.accumulatedTimespanUs() >= kPublishIntervalUs) {
			const OpticalFlowAccumulator::Window window = _accumulator.window();

			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;

			window.pixel_flow.copyTo(vehicle_optical_flow.pixel_flow);
			window.delta_angle.copyTo(vehicle_optical_flow.delta_angle);
			vehicle_optical_flow.integration_timespan_us = window.integration_timespan_us;
			vehicle_optical_flow.quality = window.quality;

			vehicle_optical_flow.distance_m = NAN;
			vehicle_optical_flow.max_flow_rate = sensor_optical_flow.max_flow_rate;
			vehicle_optical_flow.min_ground_distance = sensor_optical_flow.min_ground_distance;
			vehicle_optical_flow.max_ground_distance = sensor_optical_flow.max_ground_distance;

			vehicle_optical_flow.timestamp = hrt_absolute_time();
			_vehicle_optical_flow_pub.publish(vehicle_optical_flow);

			_accumulator.reset();
		}
	}

	// reschedule backup
	ScheduleDelayed(10_ms);

	perf_end(_cycle_perf);
}

void NodeOpticalFlow::PrintStatus()
{
	PX4_INFO_RAW("[node_optical_flow] %" PRIu32 " Hz\n", static_cast<uint32_t>(CONFIG_SENSORS_NODE_OPTICAL_FLOW_RATE));
	perf_print_counter(_cycle_perf);
}

} // namespace sensors
