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
		uORB::Manager::initialize();
		param_init();
	}

	void TearDown() override
	{
		param_reset_all();
		uORB::Manager::terminate();
	}
};

TEST_F(CollisionPreventionTest, instantiation) { CollisionPrevention cp(nullptr); }

TEST_F(CollisionPreventionTest, behaviorOff)
{
	// GIVEN: a simple setup condition
	CollisionPrevention cp(nullptr);
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
	CollisionPrevention cp(nullptr);
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3.f;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);


	// WHEN: we set the parameter check then apply the setpoint modification
	float value = 10; // try to keep 10m away from obstacles
	param_set(param, &value);

	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

	// THEN: it shouldn't interfere with the setpoint, because there isn't an obstacle
	EXPECT_EQ(original_setpoint, modified_setpoint);
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithAnObstacle)
{
	// GIVEN: a simple setup condition
	CollisionPrevention cp(nullptr);
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);

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

	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);


	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	matrix::Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: it should be cut down a lot
	EXPECT_GT(original_setpoint.norm() * 0.5f, modified_setpoint.norm()); //FIXME: this should actually be constrained to 0
}
