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

#include <gtest/gtest.h>

#include "GZBridge.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/failure_injection.h>
#include <uORB/topics/sensor_gps.h>

class GZBridgeTestPeer
{
public:
	static void navSatCallback(GZBridge &bridge, const gz::msgs::NavSat &message)
	{
		bridge.navSatCallback(message);
	}
};

namespace
{

gz::msgs::NavSat navSat(double latitude_deg, double longitude_deg)
{
	gz::msgs::NavSat message;
	message.set_latitude_deg(latitude_deg);
	message.set_longitude_deg(longitude_deg);
	message.set_altitude(500.0);
	message.set_velocity_north(1.0);
	message.set_velocity_east(2.0);
	message.set_velocity_up(0.5);
	return message;
}

failure_injection_s gpsFailure(uint8_t failure_type)
{
	failure_injection_s config{};
	config.timestamp = hrt_absolute_time();

	if (failure_type != failure_injection_s::FAILURE_TYPE_OK) {
		config.count = 1;
		config.unit[0] = failure_injection_s::FAILURE_UNIT_SENSOR_GPS;
		config.instance_mask[0] = 1;
		config.failure_type[0] = failure_type;
	}

	return config;
}

} // namespace

TEST(GZBridge, AppliesGpsFailureInjectionAtNavSatPublication)
{
	GZBridge bridge{"default", "x500"};
	uORB::Publication<failure_injection_s> failure_pub{ORB_ID(failure_injection)};
	uORB::Subscription gps_sub{ORB_ID(sensor_gps)};

	// Initialize the map reference.
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.397742, 8.545594));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.397742, 8.545594));

	sensor_gps_s baseline{};
	ASSERT_TRUE(gps_sub.update(&baseline));
	ASSERT_EQ(baseline.fix_type, sensor_gps_s::FIX_TYPE_3D);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_STUCK)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gps_s stuck{};
	ASSERT_TRUE(gps_sub.update(&stuck));
	EXPECT_DOUBLE_EQ(stuck.latitude_deg, baseline.latitude_deg);
	EXPECT_DOUBLE_EQ(stuck.longitude_deg, baseline.longitude_deg);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_WRONG)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gps_s wrong{};
	ASSERT_TRUE(gps_sub.update(&wrong));
	EXPECT_EQ(wrong.fix_type, sensor_gps_s::FIX_TYPE_2D);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_OFF)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));
	EXPECT_FALSE(gps_sub.updated());

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_OK)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gps_s recovered{};
	ASSERT_TRUE(gps_sub.update(&recovered));
	EXPECT_EQ(recovered.fix_type, sensor_gps_s::FIX_TYPE_3D);
	EXPECT_NE(recovered.latitude_deg, baseline.latitude_deg);
	EXPECT_NE(recovered.longitude_deg, baseline.longitude_deg);
}
