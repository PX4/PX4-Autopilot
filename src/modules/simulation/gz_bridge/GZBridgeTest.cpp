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
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/vehicle_odometry.h>

class GZBridgeTestPeer
{
public:
	static void navSatCallback(GZBridge &bridge, const gz::msgs::NavSat &message)
	{
		bridge.navSatCallback(message, 0);
	}

	static void odometryCallback(GZBridge &bridge, const gz::msgs::OdometryWithCovariance &message)
	{
		bridge.odometryCallback(message);
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

gz::msgs::OdometryWithCovariance odometry(double east, double north, double up)
{
	gz::msgs::OdometryWithCovariance message;
	gz::msgs::Pose *pose = message.mutable_pose_with_covariance()->mutable_pose();
	pose->mutable_position()->set_x(east);
	pose->mutable_position()->set_y(north);
	pose->mutable_position()->set_z(up);
	pose->mutable_orientation()->set_w(1.0);
	message.mutable_twist_with_covariance()->mutable_twist()->mutable_linear()->set_x(1.0);

	for (int i = 0; i < 36; i++) {
		message.mutable_pose_with_covariance()->mutable_covariance()->add_data(0.01);
		message.mutable_twist_with_covariance()->mutable_covariance()->add_data(0.01);
	}

	return message;
}

failure_injection_s sensorFailure(uint8_t unit, uint8_t failure_type)
{
	failure_injection_s config{};
	config.timestamp = hrt_absolute_time();

	if (failure_type != failure_injection_s::FAILURE_TYPE_OK) {
		config.count = 1;
		config.unit[0] = unit;
		config.instance_mask[0] = 1;
		config.failure_type[0] = failure_type;
	}

	return config;
}

failure_injection_s gpsFailure(uint8_t failure_type)
{
	return sensorFailure(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, failure_type);
}

failure_injection_s vioFailure(uint8_t failure_type)
{
	return sensorFailure(failure_injection_s::FAILURE_UNIT_SENSOR_VIO, failure_type);
}

} // namespace

TEST(GZBridge, AppliesGpsFailureInjectionAtNavSatPublication)
{
	GZBridge bridge{"default", "x500"};
	uORB::Publication<failure_injection_s> failure_pub{ORB_ID(failure_injection)};
	uORB::Subscription sensor_gnss_sub{ORB_ID(sensor_gnss)};

	// Initialize the map reference.
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.397742, 8.545594));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.397742, 8.545594));

	sensor_gnss_s baseline{};
	ASSERT_TRUE(sensor_gnss_sub.update(&baseline));
	ASSERT_EQ(baseline.fix_type, sensor_gnss_s::FIX_TYPE_3D);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_STUCK)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gnss_s stuck{};
	ASSERT_TRUE(sensor_gnss_sub.update(&stuck));
	EXPECT_DOUBLE_EQ(stuck.latitude, baseline.latitude);
	EXPECT_DOUBLE_EQ(stuck.longitude, baseline.longitude);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_WRONG)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gnss_s wrong{};
	ASSERT_TRUE(sensor_gnss_sub.update(&wrong));
	EXPECT_EQ(wrong.fix_type, sensor_gnss_s::FIX_TYPE_2D);

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_OFF)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));
	EXPECT_FALSE(sensor_gnss_sub.updated());

	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_OK)));
	GZBridgeTestPeer::navSatCallback(bridge, navSat(47.407742, 8.555594));

	sensor_gnss_s recovered{};
	ASSERT_TRUE(sensor_gnss_sub.update(&recovered));
	EXPECT_EQ(recovered.fix_type, sensor_gnss_s::FIX_TYPE_3D);
	EXPECT_NE(recovered.latitude, baseline.latitude);
	EXPECT_NE(recovered.longitude, baseline.longitude);
}

TEST(GZBridge, AppliesVioFailureInjectionAtOdometryPublication)
{
	GZBridge bridge{"default", "x500"};
	uORB::Publication<failure_injection_s> failure_pub{ORB_ID(failure_injection)};
	uORB::Subscription visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};

	// gz odometry is ENU; the bridge publishes NED (north, east, down).
	GZBridgeTestPeer::odometryCallback(bridge, odometry(1.0, 2.0, 3.0));

	vehicle_odometry_s baseline{};
	ASSERT_TRUE(visual_odometry_sub.update(&baseline));
	ASSERT_FLOAT_EQ(baseline.position[0], 2.f);
	ASSERT_FLOAT_EQ(baseline.position[1], 1.f);
	ASSERT_FLOAT_EQ(baseline.position[2], -3.f);

	ASSERT_TRUE(failure_pub.publish(vioFailure(failure_injection_s::FAILURE_TYPE_STUCK)));
	GZBridgeTestPeer::odometryCallback(bridge, odometry(11.0, 12.0, 13.0));

	// Stuck keeps publishing, with fresh timestamps, the last good sample.
	vehicle_odometry_s stuck{};
	ASSERT_TRUE(visual_odometry_sub.update(&stuck));
	EXPECT_FLOAT_EQ(stuck.position[0], baseline.position[0]);
	EXPECT_FLOAT_EQ(stuck.position[1], baseline.position[1]);
	EXPECT_FLOAT_EQ(stuck.position[2], baseline.position[2]);
	EXPECT_GE(stuck.timestamp_sample, baseline.timestamp_sample);

	ASSERT_TRUE(failure_pub.publish(vioFailure(failure_injection_s::FAILURE_TYPE_OFF)));
	GZBridgeTestPeer::odometryCallback(bridge, odometry(11.0, 12.0, 13.0));
	EXPECT_FALSE(visual_odometry_sub.updated());

	// A GPS failure must not affect visual odometry.
	ASSERT_TRUE(failure_pub.publish(gpsFailure(failure_injection_s::FAILURE_TYPE_OFF)));
	GZBridgeTestPeer::odometryCallback(bridge, odometry(11.0, 12.0, 13.0));

	vehicle_odometry_s unaffected{};
	ASSERT_TRUE(visual_odometry_sub.update(&unaffected));
	EXPECT_FLOAT_EQ(unaffected.position[0], 12.f);

	ASSERT_TRUE(failure_pub.publish(vioFailure(failure_injection_s::FAILURE_TYPE_OK)));
	GZBridgeTestPeer::odometryCallback(bridge, odometry(21.0, 22.0, 23.0));

	vehicle_odometry_s recovered{};
	ASSERT_TRUE(visual_odometry_sub.update(&recovered));
	EXPECT_FLOAT_EQ(recovered.position[0], 22.f);
	EXPECT_FLOAT_EQ(recovered.position[1], 21.f);
	EXPECT_FLOAT_EQ(recovered.position[2], -23.f);
}
