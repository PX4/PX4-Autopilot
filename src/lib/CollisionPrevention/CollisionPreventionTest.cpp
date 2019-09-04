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
hrt_abstime mocked_time = 0;

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
	obstacle_distance_s &getObstacleMap() {return _obstacle_map_body_frame;}
	void test_addDistanceSensorData(distance_sensor_s &distance_sensor, const matrix::Quatf &attitude)
	{
		_addDistanceSensorData(distance_sensor, attitude);
	}
	void test_addObstacleSensorData(const obstacle_distance_s &obstacle, const matrix::Quatf &attitude)
	{
		_addObstacleSensorData(obstacle, attitude);
	}
};

class TestTimingCollisionPrevention : public TestCollisionPrevention
{
public:
	TestTimingCollisionPrevention() : TestCollisionPrevention() {}
protected:
	hrt_abstime getTime() override
	{
		return mocked_time;
	}

	hrt_abstime getElapsedTime(const hrt_abstime *ptr) override
	{
		return mocked_time - *ptr;
	}
};

TEST_F(CollisionPreventionTest, instantiation) { CollisionPrevention cp(nullptr); }

TEST_F(CollisionPreventionTest, behaviorOff)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;

	// THEN: the collision prevention should be turned off by default
	EXPECT_FALSE(cp.is_active());
}

TEST_F(CollisionPreventionTest, noSensorData)
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

	// THEN: collision prevention should be enabled and limit the speed to zero
	EXPECT_TRUE(cp.is_active());
	EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm());
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithObstacleMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint1(10, 0);
	matrix::Vector2f original_setpoint2(-10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			message.distances[i] = 101;

		} else {
			message.distances[i] = 10001;
		}

	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);
	matrix::Vector2f modified_setpoint1 = original_setpoint1;
	matrix::Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, max_speed, curr_pos, curr_vel);
	cp.modifySetpoint(modified_setpoint2, max_speed, curr_pos, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the velocity setpoint should be cut down to zero
	// case 2: the velocity setpoint should stay the same as the input
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_FLOAT_EQ(0.f, modified_setpoint1.norm()) << modified_setpoint1(0) << "," << modified_setpoint1(1);
	EXPECT_EQ(original_setpoint2, modified_setpoint2);
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithDistanceMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint1(10, 0);
	matrix::Vector2f original_setpoint2(-10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	distance_sensor_s message;
	message.timestamp = hrt_absolute_time();
	message.min_distance = 1.f;
	message.max_distance = 100.f;
	message.current_distance = 1.1f;

	message.variance = 0.1f;
	message.signal_quality = 100;
	message.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	message.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	message.h_fov = math::radians(50.f);
	message.v_fov = math::radians(30.f);

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(distance_sensor), distance_sensor_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	//WHEN:  We run the setpoint modification
	matrix::Vector2f modified_setpoint1 = original_setpoint1;
	matrix::Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, max_speed, curr_pos, curr_vel);
	cp.modifySetpoint(modified_setpoint2, max_speed, curr_pos, curr_vel);
	orb_unadvertise(distance_sensor_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the velocity setpoint should be cut down to zero because there is an obstacle
	// case 2: the velocity setpoint should be cut down to zero because there is no data
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_FLOAT_EQ(0.f, modified_setpoint1.norm()) << modified_setpoint1(0) << "," << modified_setpoint1(1);
	EXPECT_FLOAT_EQ(0.f, modified_setpoint2.norm()) << modified_setpoint2(0) << "," << modified_setpoint2(1);
}

TEST_F(CollisionPreventionTest, testPurgeOldData)
{
	// GIVEN: a simple setup condition
	hrt_abstime start_time = hrt_absolute_time();
	mocked_time = start_time;
	TestTimingCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = start_time;
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::MPC_COL_PREV_D);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message, message_empty;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = start_time;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;
	message_empty = message;

	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			message.distances[i] = 10001;

		} else {
			message.distances[i] = UINT16_MAX;
		}

		message_empty.distances[i] = UINT16_MAX;
	}


	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	for (int i = 0; i < 10; i++) {

		matrix::Vector2f modified_setpoint = original_setpoint;
		cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

		mocked_time = mocked_time + 100000; //advance time by 0.1 seconds
		message_empty.timestamp = mocked_time;
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message_empty);

		if (i < 6) {
			// THEN: If the data is new enough, the velocity setpoint should stay the same as the input
			EXPECT_EQ(original_setpoint, modified_setpoint);

		} else {
			// THEN: If the data is expired, the velocity setpoint should be cut down to zero because there is no data
			EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm()) << modified_setpoint(0) << "," << modified_setpoint(1);
		}
	}

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
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
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
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
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360.f / distances_array_size;

	//fov from 45deg to 225deg
	for (int i = 0; i < distances_array_size; i++) {
		float angle = i * message.increment;

		if (angle > 45.f && angle < 225.f) {
			message.distances[i] = 700;

		} else {
			message.distances[i] = UINT16_MAX;
		}
	}

	// WHEN: we publish the message and modify the setpoint for different demanded setpoints
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);

	for (int i = 0; i < distances_array_size; i++) {

		float angle_deg = (float)i * message.increment;
		float angle_rad = math::radians(angle_deg);
		matrix::Vector2f original_setpoint = {10.f *(float)cos(angle_rad), 10.f *(float)sin(angle_rad)};
		matrix::Vector2f modified_setpoint = original_setpoint;
		message.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
		cp.modifySetpoint(modified_setpoint, max_speed, curr_pos, curr_vel);

		if (angle_deg > 50.f && angle_deg < 230.f) {
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

TEST_F(CollisionPreventionTest, jerkLimit)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	matrix::Vector2f original_setpoint(10, 0);
	float max_speed = 3;
	matrix::Vector2f curr_pos(0, 0);
	matrix::Vector2f curr_vel(2, 0);

	// AND: distance set to 5m
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
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 700;
	}

	// AND: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	matrix::Vector2f modified_setpoint_default_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_default_jerk, max_speed, curr_pos, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// AND: we now set max jerk to 0.1
	param = param_handle(px4::params::MPC_JERK_MAX);
	value = 0.1; // 0.1 maximum jerk
	param_set(param, &value);
	cp.paramsChanged();

	// WHEN: we run the setpoint modification again
	matrix::Vector2f modified_setpoint_limited_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_limited_jerk, max_speed, curr_pos, curr_vel);

	// THEN: the new setpoint should be much slower than the one with default jerk
	EXPECT_LT(modified_setpoint_limited_jerk.norm() * 10, modified_setpoint_default_jerk.norm());
}

TEST_F(CollisionPreventionTest, addDistanceSensorData)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	cp.getObstacleMap().increment = 10.f;
	matrix::Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;

	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (uint32_t i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add distance sensor data to the right
	distance_sensor.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
	distance_sensor.h_fov = math::radians(19.99f);
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 8 || i == 9) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}
	}

	//WHEN: we add additionally distance sensor data to the left
	distance_sensor.orientation = distance_sensor_s::ROTATION_LEFT_FACING;
	distance_sensor.h_fov = math::radians(50.f);
	distance_sensor.current_distance = 8.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 8 || i == 9) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else if (i >= 24 && i <= 29) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}
	}

	//WHEN: we add additionally distance sensor data to the front
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(10.1f);
	distance_sensor.current_distance = 3.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 8 || i == 9) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else if (i >= 24 && i <= 29) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800);

		} else if (i == 0) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 300);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}
	}


}

TEST_F(CollisionPreventionTest, addObstacleSensorData_attitude)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	cp.getObstacleMap().increment = 10.f;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	matrix::Quaternion<float> vehicle_attitude1(1, 0, 0, 0); //unit transform
	matrix::Euler<float> attitude2_euler(0, 0, M_PI / 2.0);
	matrix::Quaternion<float> vehicle_attitude2(attitude2_euler); //90 deg yaw
	matrix::Euler<float> attitude3_euler(0, 0, -M_PI / 4.0);
	matrix::Quaternion<float> vehicle_attitude3(attitude3_euler); // -45 deg yaw
	matrix::Euler<float> attitude4_euler(0, 0, M_PI);
	matrix::Quaternion<float> vehicle_attitude4(attitude4_euler); // 180 deg yaw

	//obstacle at 10-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 2; i < 6 ; i++) {
		obstacle_msg.distances[i] = 500;
	}


	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add distance sensor data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude1);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 1 || i == 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}


	//WHEN: we add distance sensor data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude2);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 28 || i == 29) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add distance sensor data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude3);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 6 || i == 7) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add distance sensor data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude4);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 19 || i == 20) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, addObstacleSensorData_bodyframe)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	cp.getObstacleMap().increment = 10.f;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	matrix::Quaternion<float> vehicle_attitude1(1, 0, 0, 0); //unit transform
	matrix::Euler<float> attitude2_euler(0, 0, M_PI / 2.0);
	matrix::Quaternion<float> vehicle_attitude2(attitude2_euler); //90 deg yaw
	matrix::Euler<float> attitude3_euler(0, 0, -M_PI / 4.0);
	matrix::Quaternion<float> vehicle_attitude3(attitude3_euler); // -45 deg yaw
	matrix::Euler<float> attitude4_euler(0, 0, M_PI);
	matrix::Quaternion<float> vehicle_attitude4(attitude4_euler); // 180 deg yaw

	//obstacle at 10-30 deg body frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 2; i < 6 ; i++) {
		obstacle_msg.distances[i] = 500;
	}


	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add obstacle data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude1);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 1 || i == 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude2);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 1 || i == 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude3);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 1 || i == 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude4);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i == 1 || i == 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}


TEST_F(CollisionPreventionTest, addObstacleSensorData_resolution_offset)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	cp.getObstacleMap().increment = 10.f;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 6.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	matrix::Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform

	//obstacle at 0-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i < 5 ; i++) {
		obstacle_msg.distances[i] = 500;
	}

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 0 && i <= 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add distance sensor data with an angle offset
	obstacle_msg.angle_offset = 30.f;
	cp.test_addObstacleSensorData(obstacle_msg, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 3 && i <= 5) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}
