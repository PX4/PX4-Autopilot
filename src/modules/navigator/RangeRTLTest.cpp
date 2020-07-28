#define MODULE_NAME "Navigator"

#include "navigator.h"

#include "rtl.h"

#include <gtest/gtest.h>

TEST(Navigator_and_RTL, compiles_woohoooo)
{
	Navigator n;
	RTL rtl(&n);

	rtl.find_RTL_destination();
}

class RangeRTL_tth : public ::testing::Test
{
public:
	matrix::Vector3f vehicle_local_pos ;
	matrix::Vector3f rtl_point_local_pos ;
	matrix::Vector2f wind_vel;
	float vehicle_speed;
	float vehicle_descent_speed;

	void SetUp() override
	{
		vehicle_local_pos  = matrix::Vector3f(0, 0, 0);
		rtl_point_local_pos  = matrix::Vector3f(0, 0, 0);
		wind_vel  = matrix::Vector2f(0, 0);
		vehicle_speed = 5;
		vehicle_descent_speed = 1;
	}
};

TEST_F(RangeRTL_tth, zero_distance_zero_time)
{
	// GIVEN: zero distances (defaults)

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be zero
	EXPECT_FLOAT_EQ(tth, 0.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 6.2f;
	vehicle_local_pos(0) = vehicle_local_pos(1) = (vehicle_speed * 10) / sqrtf(2);

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be zero
	EXPECT_FLOAT_EQ(tth, 10.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_5_seconds_z)
{
	// GIVEN: 10 seconds of xy distance and 5 seconds of Z
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	vehicle_local_pos(0) = vehicle_local_pos(1) = (vehicle_speed * 10) / sqrtf(2);
	vehicle_local_pos(2) = vehicle_descent_speed * 5;

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 15 seconds
	EXPECT_FLOAT_EQ(tth, 15.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_downwind_to_home)
{
	// GIVEN: 10 seconds of xy distance and 5 seconds of Z, and the wind is towards home
	vehicle_speed = 4.2f;
	vehicle_local_pos(0) = vehicle_local_pos(1) = (vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(-1, -1);

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 10, because we don't rely on wind towards home for RTL
	EXPECT_FLOAT_EQ(tth, 10.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_upwind_to_home)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	vehicle_local_pos(0) = vehicle_local_pos(1) = (vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(1, 1) / sqrt(2) * vehicle_speed / 10;

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 11.111111... because it slows us down by 10% and time = dist/speed
	EXPECT_FLOAT_EQ(tth, 10 / 0.9f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_z_wind_across_home)
{
	// GIVEN: a 3 4 5 triangle, with vehicle airspeed being 5, wind 3, ground speed 4
	// and the vehicle is 10 seconds away

	vehicle_speed = 5.f;
	wind_vel = matrix::Vector2f(-1, 1) / sqrt(2) * 3.;
	vehicle_local_pos(0) = vehicle_local_pos(1) = (4 * 10) / sqrtf(2);

	// WHEN: we get the tth
	float tth = time_to_home(vehicle_local_pos, rtl_point_local_pos, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 10
	EXPECT_FLOAT_EQ(tth, 10);
}
