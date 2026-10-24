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

/**
 * Test for VehicleGPSPosition
 */

#include <gtest/gtest.h>

#include "VehicleGPSPosition.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/uORBManager.hpp>

namespace sensors
{

// reaches the private state the tests set up and assert on
class VehicleGPSPositionTestPeer
{
public:
	static constexpr uint32_t kDeviceIds[VehicleGPSPosition::GPS_MAX_RECEIVERS] {0x3e4f01, 0x3e4f02};

	// slot 0 configured as SENS_GNSS0_HDG Two receivers 1 m from slot 1, which is disabled, with receivers whose
	// checks pass and whose latest fixes are both new
	static void setupTwoReceivers(VehicleGPSPosition &module)
	{
		module._two_receivers_slot = 0;

		for (int i = 0; i < VehicleGPSPosition::GPS_MAX_RECEIVERS; i++) {
			VehicleGPSPosition::GpsParamSlot &slot = module._gnss_param_slots[i];
			slot.device_id = kDeviceIds[i];
			slot.baseline_length = (i == 0) ? 1.f : 0.f;
			slot.heading_enabled = (i == 0);
			slot.two_receivers = (i == 0);

			// with every check disabled and no health time, the second run passes
			GnssChecks::Params params{};
			params.check_mask = 0;
			params.min_health_time_us = 0;
			module._gnss_checks[i].setParams(params);
			gnssChecksSample checks_sample{};
			checks_sample.time_us = 1;
			module._gnss_checks[i].run(checks_sample, false, false, true);
			checks_sample.time_us = 2;
			module._gnss_checks[i].run(checks_sample, false, false, true);

			sensor_gnss_s &gnss = module._latest_sample[i];
			gnss.timestamp = hrt_absolute_time();
			gnss.timestamp_sample = gnss.timestamp;
			gnss.device_id = kDeviceIds[i];
			gnss.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
			gnss.time_utc_usec = 1760000000000000;
			gnss.eph = 0.01f;

			module._two_receiver_gnss_updated[i] = true;
		}
	}

	static sensor_gnss_s &latestSample(VehicleGPSPosition &module, int instance) { return module._latest_sample[instance]; }
	static bool checksPassed(const VehicleGPSPosition &module, int instance) { return module._gnss_checks[instance].passed(); }

	// the source the heading is held on, settling since settled_since
	static void setHeadingSource(VehicleGPSPosition &module, uint32_t device_id, hrt_abstime settled_since)
	{
		module._heading_source = {device_id, hrt_absolute_time(), settled_since};
	}

	static uint32_t headingSourceDeviceId(const VehicleGPSPosition &module) { return module._heading_source.device_id; }
	static hrt_abstime settledSince(const VehicleGPSPosition &module) { return module._heading_source.settled_since; }

	static void updateTwoReceiverHeading(VehicleGPSPosition &module) { module.UpdateTwoReceiverHeading(); }
};

} // namespace sensors

using sensors::VehicleGPSPosition;
using Peer = sensors::VehicleGPSPositionTestPeer;

class VehicleGPSPositionTest : public ::testing::Test
{
public:
	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
	}
};

TEST_F(VehicleGPSPositionTest, FailedBaselineRestartsHeadingSettle)
{
	// GIVEN: two receivers configured for a two receiver heading, and the heading held on receiver 0 while it settles
	VehicleGPSPosition module;
	Peer::setupTwoReceivers(module);
	ASSERT_TRUE(Peer::checksPassed(module, 0));
	ASSERT_TRUE(Peer::checksPassed(module, 1));

	Peer::setHeadingSource(module, Peer::kDeviceIds[0], hrt_absolute_time());
	ASSERT_NE(Peer::settledSince(module), 0u);

	// WHEN: receiver 1 loses its RTK fixed solution, so no baseline can be measured
	Peer::latestSample(module, 1).fix_type = sensor_gnss_s::FIX_TYPE_RTK_FLOAT;
	Peer::updateTwoReceiverHeading(module);

	// THEN: the source is kept, but has to settle again once the baseline is measured again
	EXPECT_EQ(Peer::headingSourceDeviceId(module), Peer::kDeviceIds[0]);
	EXPECT_EQ(Peer::settledSince(module), 0u);
}
