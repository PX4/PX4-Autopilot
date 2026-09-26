#include <gtest/gtest.h>

#include "TrajectoryConstraints.hpp"

using namespace matrix;
using namespace math::trajectory;

class TrajectoryConstraintsTest : public ::testing::Test
{
public:
	VehicleDynamicLimits config;

	Vector3f vehicle_location;
	Vector3f target;
	Vector3f next_target;

	float final_speed = 0;

	void SetUp() override
	{
		config.z_accept_rad = 1.f;
		config.xy_accept_rad = 0.99f;

		config.max_acc_xy = 3.f;
		config.max_jerk = 10.f;

		config.max_speed_xy = 10.f;

		config.max_acc_xy_radius_scale = 0.8f;

		/*
		 *             (20,20)
		 *              Next target
		 *
		 *              ^
		 *              |
		 *
		 * (10,10)      (20,10)
		 * Vehicle  ->  Target
		 *
		 */
		vehicle_location = Vector3f(10, 10, 5);
		target = Vector3f(20, 10, 5);
		next_target = Vector3f(20, 20, 5);
	}
};

TEST_F(TrajectoryConstraintsTest, testStraight)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.5f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightNaN)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.5f * (next_target - vehicle_location);
	next_target(0) = NAN;
	next_target(1) = NAN;

	// WHEN: we get the speed for points which are NaN afterwards
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed to the closer point
	Vector3f direct_points[2] = {vehicle_location, target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightLowJerkClose)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.05f * (next_target - vehicle_location);
	config.max_jerk = 8.f;

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightMidClose)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.05f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightMidFar)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.95f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}


TEST_F(TrajectoryConstraintsTest, test90Angle)
{
	// GIVEN: 3 waypoints in 90 degree angle
	EXPECT_FLOAT_EQ(0.f, (vehicle_location - target).dot(target - next_target));

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed); //faster
	EXPECT_LT(through_speed, stop_speed * 1.03f); // but less than 3% faster
}

TEST_F(TrajectoryConstraintsTest, test45Angle)
{
	// GIVEN: 3 waypoints in 45 degree angle
	next_target = Vector3f(25, 15, 5);

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed * 1.03f); // more than 3% faster
	EXPECT_LT(through_speed, stop_speed * 1.06f); // but less than 6% faster
}

TEST_F(TrajectoryConstraintsTest, test10Angle)
{
	// GIVEN: 3 waypoints in 10 degree angle
	next_target = Vector3f(30, 11.7, 5);

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed * 1.25f); // more than 25% faster
	EXPECT_LT(through_speed, stop_speed * 1.3f); // but less than 30% faster
}

TEST_F(TrajectoryConstraintsTest, test10AngleFarNext)
{
	// GIVEN: 3 waypoints in 10 degree angle, but next waypoint is far
	next_target = 2.f * (Vector3f(30, 11.7, 5) - target) + target;

	// WHEN: we get the speed for travel around the path
	Vector3f far_waypoints[3] = {vehicle_location, target, next_target};
	float far_speed = computeXYSpeedFromWaypoints<3>(far_waypoints, config);

	// THEN: it should be the same speed as a closer next waypoint at the same angle, since the bottleneck is the turn
	next_target = Vector3f(30, 11.7, 5);
	Vector3f close_waypoints[3] = {vehicle_location, target, next_target};
	float close_speed = computeXYSpeedFromWaypoints<3>(close_waypoints, config);

	EXPECT_FLOAT_EQ(far_speed, close_speed);
}

TEST_F(TrajectoryConstraintsTest, test10AngleCloseNext)
{
	// GIVEN: 3 waypoints in right angle, but next waypoint is far
	next_target = .2f * (Vector3f(30, 11.7, 5) - target) + target;

	// WHEN: we get the speed for travel around the path
	Vector3f close_waypoints[3] = {vehicle_location, target, next_target};
	float close_speed = computeXYSpeedFromWaypoints<3>(close_waypoints, config);

	// THEN: it should be slower than a further next waypoint at the same angle, since the bottleneck is the distance
	next_target = Vector3f(30, 11.7, 5);
	Vector3f normal_waypoints[3] = {vehicle_location, target, next_target};
	float normal_speed = computeXYSpeedFromWaypoints<3>(normal_waypoints, config);

	EXPECT_LT(close_speed, normal_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightNextInsideAcceptanceRadius)
{
	// GIVEN: 3 waypoints in straight line, the next one closer to the target than the acceptance radius
	next_target = target + 0.5f * (target - vehicle_location).unit_or_zero();
	EXPECT_LT((next_target - target).norm(), config.xy_accept_rad);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: the target must not be treated as a stop, only the (short) remaining distance to the next waypoint counts
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_GT(through_speed, stop_speed);
	EXPECT_LE(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightNextInsideAcceptanceRadiusWithExitSpeed)
{
	// GIVEN: a straight line where the waypoint after next is far, but next is inside the acceptance radius of the target
	// (e.g. the entry point of a survey followed by the first survey line)
	const Vector3f direction = (target - vehicle_location).unit_or_zero();
	next_target = target + 0.5f * direction;
	const Vector3f after_next = target + 100.f * direction;

	// WHEN: we get the speed knowing the waypoint after next
	Vector3f waypoints[4] = {vehicle_location, target, next_target, after_next};
	float through_speed = computeXYSpeedFromWaypoints<4>(waypoints, config);

	// THEN: the vehicle can fly through both waypoints at cruise speed
	EXPECT_FLOAT_EQ(through_speed, config.max_speed_xy);
}

TEST_F(TrajectoryConstraintsTest, test90AngleNextInsideAcceptanceRadius)
{
	// GIVEN: a 90 degree corner onto a segment shorter than the acceptance radius
	next_target = target + 0.5f * (next_target - target).unit_or_zero();
	EXPECT_FLOAT_EQ(0.f, (vehicle_location - target).dot(target - next_target));

	// WHEN: we get the speed for travel around the corner
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it is at least as fast as stopping at the corner, but slower than the same corner onto a long segment
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);
	Vector3f long_waypoints[3] = {vehicle_location, target, Vector3f(20, 20, 5)};
	float long_speed = computeXYSpeedFromWaypoints<3>(long_waypoints, config);

	EXPECT_GE(through_speed, stop_speed);
	EXPECT_LT(through_speed, long_speed);
}

TEST_F(TrajectoryConstraintsTest, testHairpinNextInsideAcceptanceRadius)
{
	// GIVEN: a 180 degree turn onto a segment shorter than the acceptance radius
	next_target = target - 0.5f * (target - vehicle_location).unit_or_zero();

	// WHEN: we get the speed for travel around the hairpin
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: the vehicle has to stop at the turn
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_FLOAT_EQ(through_speed, stop_speed);
}

TEST_F(TrajectoryConstraintsTest, testStopAtLastMatchesFixedSizeVersion)
{
	// GIVEN: the 90 degree corner of the fixture
	Vector3f waypoints[3] = {vehicle_location, target, next_target};

	// WHEN: we get the speed with a zero and with an unknown velocity after the last waypoint
	float stop_speed = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config);
	float unknown_speed = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{NAN, NAN, NAN}, config);

	// THEN: both assume a stop at the last waypoint, same as the fixed size version
	EXPECT_FLOAT_EQ(stop_speed, computeXYSpeedFromWaypoints<3>(waypoints, config));
	EXPECT_FLOAT_EQ(unknown_speed, stop_speed);
}

TEST_F(TrajectoryConstraintsTest, testVelocityAfterLastCarriesSpeedThroughLast)
{
	// GIVEN: a survey-like pattern: target and a collinear next waypoint 15m behind it, the path continuing straight
	config.max_jerk = 4.f;
	config.max_speed_xy = 15.f;
	config.xy_accept_rad = 10.f;
	vehicle_location = Vector3f(70, 0, 5); // 10m before the target, i.e. inside the braking zone
	target = Vector3f(80, 0, 5);
	next_target = Vector3f(95, 0, 5);
	Vector3f waypoints[3] = {vehicle_location, target, next_target};

	// WHEN: we get the speed without knowing what follows the next waypoint
	float speed_with_stop = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config);

	// THEN: the vehicle has to plan a stop 15m after the target, which caps the speed at the target well below cruise
	float stop_in_15m_speed = computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, 15.f, 0.f);
	EXPECT_NEAR(stop_in_15m_speed, 6.f, 0.01f);
	EXPECT_LT(speed_with_stop, config.max_speed_xy);

	// WHEN: we get the speed knowing the vehicle may leave the next waypoint at cruise speed along the same line
	float speed_through = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{config.max_speed_xy, 0.f, 0.f}, config);

	// THEN: the straight line can be flown at cruise speed
	EXPECT_GT(speed_through, speed_with_stop);
	EXPECT_FLOAT_EQ(speed_through, config.max_speed_xy);
}

TEST_F(TrajectoryConstraintsTest, testVelocityAfterLastNormIsAnUpperBound)
{
	// GIVEN: the same straight pattern, but the mission only allows 2m/s after the next waypoint
	config.max_jerk = 4.f;
	config.max_speed_xy = 15.f;
	config.xy_accept_rad = 10.f;
	vehicle_location = Vector3f(70, 0, 5);
	target = Vector3f(80, 0, 5);
	next_target = Vector3f(95, 0, 5);
	Vector3f waypoints[3] = {vehicle_location, target, next_target};

	// WHEN: we get the speed with that constraint
	const float exit_speed = 2.f;
	float speed_limited = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{exit_speed, 0.f, 0.f}, config);

	// THEN: it lies between stopping at next and passing next at cruise speed, and it is exactly the speed which
	// lets the vehicle brake down to the constraint over the remaining distance
	float speed_with_stop = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config);
	float speed_through = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{config.max_speed_xy, 0.f, 0.f}, config);
	EXPECT_GT(speed_limited, speed_with_stop);
	EXPECT_LT(speed_limited, speed_through);

	float speed_at_target = computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, 15.f, exit_speed);
	float expected = computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, 10.f, speed_at_target);
	EXPECT_FLOAT_EQ(speed_limited, expected);
}

TEST_F(TrajectoryConstraintsTest, testVelocityAfterLastDirectionLimitsTurnAtLast)
{
	// GIVEN: target, a close collinear next waypoint and a path turning by 90 degrees right after it
	config.max_jerk = 4.f;
	config.max_speed_xy = 15.f;
	config.xy_accept_rad = 10.f;
	vehicle_location = Vector3f(70, 0, 5);
	target = Vector3f(80, 0, 5);
	next_target = Vector3f(95, 0, 5);
	Vector3f waypoints[3] = {vehicle_location, target, next_target};

	// WHEN: we get the speed with the path continuing straight and with it turning
	float speed_straight = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{config.max_speed_xy, 0.f, 0.f}, config);
	float speed_turning = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{0.f, config.max_speed_xy, 0.f}, config);
	float speed_with_stop = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config);

	// THEN: the turn at the next waypoint still limits the speed well below cruise, the constraint only removes the
	// full-stop assumption there
	EXPECT_GT(speed_turning, speed_with_stop);
	EXPECT_LT(speed_turning, speed_straight);
	EXPECT_LT(speed_turning, 0.5f * config.max_speed_xy);

	// AND: the same as if the corner were given as an explicit waypoint after next far enough to not limit the speed
	Vector3f corner_waypoints[4] = {vehicle_location, target, next_target, next_target + Vector3f(0, 100, 0)};
	float speed_corner = computeXYSpeedFromWaypoints(corner_waypoints, 4, Vector3f{}, config);
	EXPECT_NEAR(speed_turning, speed_corner, 1e-3f);
}

TEST_F(TrajectoryConstraintsTest, testAcceptanceRadiusPerWaypoint)
{
	// GIVEN: the 90 degree corner of the fixture, with a larger acceptance radius at the target than the default
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	const float radii[3] = {config.xy_accept_rad, 3.f * config.xy_accept_rad, config.xy_accept_rad};

	// WHEN: we get the speed with and without the per-waypoint radii
	float default_speed = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config);
	float wide_corner_speed = computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config, radii);

	// THEN: the wider turn circle at the target allows more speed
	EXPECT_GT(wide_corner_speed, default_speed);

	// AND: the radius of the last waypoint does not matter when the vehicle stops there
	const float last_radii[3] = {config.xy_accept_rad, config.xy_accept_rad, 3.f * config.xy_accept_rad};
	EXPECT_FLOAT_EQ(computeXYSpeedFromWaypoints(waypoints, 3, Vector3f{}, config, last_radii), default_speed);
}
