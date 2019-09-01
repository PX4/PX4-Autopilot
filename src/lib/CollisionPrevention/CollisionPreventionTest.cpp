/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <CollisionPrevention/CollisionPrevention.hpp>

// to run: make tests TESTFILTER=CollisionPrevention

class CollisionPreventionTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_reset_all();
	}
};

class TestCollisionPrevention : public CollisionPrevention
{
public:
	TestCollisionPrevention() : CollisionPrevention(nullptr) {}
	void paramsChanged() {CollisionPrevention::updateParamsImpl();}
};

TEST_F(CollisionPreventionTest, instantiation) { CollisionPrevention cp(nullptr); }

TEST_F(CollisionPreventionTest, behaviorOff)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3.f;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// WHEN: we check if the setpoint should be modified
	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

	// THEN: it should be the same
	EXPECT_EQ(original_setpoint, modified_setpoint);
}

TEST_F(CollisionPreventionTest, withoutObstacleMessageNothing)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3.f;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);


	// WHEN: we set the parameter check then apply the setpoint modification
	float value = 10; // try to keep 10m away from obstacles
	param_set(param, &value);
	cp.paramsChanged();

	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

	// THEN: it shouldn't interfere with the setpoint, because there isn't an obstacle
	EXPECT_EQ(original_setpoint, modified_setpoint);
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithAnObstacle)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 100;
	message.max_distance = 1000;
	message.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 101;
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: it should be cut down to zero
	EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm()) << modified_setpoint(0) << "," << modified_setpoint(1);
}

TEST_F(CollisionPreventionTest, noBias)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 5; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 100;
	message.max_distance = 2000;
	message.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 700;
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: setpoint should go into the same direction as the stick input
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(0), modified_setpoint.normalized()(0));
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(1), modified_setpoint.normalized()(1));
}

TEST_F(CollisionPreventionTest, outsideFOV)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 5; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	//fov from i = 1/4 * distances_array_size to i = 3/4 * distances_array_size
	for (int i = 0; i < distances_array_size; i++) {
		if (i > 0.25f * distances_array_size && i < 0.75f * distances_array_size) {
			message.distances[i] = 700;

		} else {
			message.distances[i] = UINT16_MAX;
		}
	}

	// WHEN: we publish the message and modify the setpoint for different demanded setpoints
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);

	for (int i = 0; i < distances_array_size; i++) {

		float angle = math::radians((float)i * message.increment);
		matrix::Vector2f original_setpoint = {10.f *(float)cos(angle), 10.f *(float)sin(angle)};
		matrix::Vector2f modified_setpoint = original_setpoint;
		message.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
		cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

		if (i > 0.25f * distances_array_size && i < 0.75f * distances_array_size) {
			// THEN: inside the FOV the setpoint should be limited
			EXPECT_GT(modified_setpoint.norm(), 0.f);
			EXPECT_LT(modified_setpoint.norm(), 10.f);

		} else {
			// THEN: outside the FOV the setpoint should be clamped to zero
			EXPECT_FLOAT_EQ(modified_setpoint.norm(), 0.f);
		}

	}

	orb_unadvertise(obstacle_distance_pub);
}
