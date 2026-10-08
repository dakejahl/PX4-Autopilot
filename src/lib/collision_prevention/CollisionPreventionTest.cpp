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
#include "CollisionPrevention.hpp"

using namespace matrix;

// to run: make tests TESTFILTER=CollisionPrevention
hrt_abstime mocked_time = 0;
const uint bin_size = CollisionPrevention::BIN_SIZE;
const uint bin_count = CollisionPrevention::BIN_COUNT;

class CollisionPreventionTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};

class TestCollisionPrevention : public CollisionPrevention
{
public:
	TestCollisionPrevention() : CollisionPrevention(nullptr) {}
	void paramsChanged() {CollisionPrevention::updateParamsImpl();}
	obstacle_distance_s &getObstacleMap() {return _obstacle_map_body_frame;}
	void test_addDistanceSensorData(distance_sensor_s &distance_sensor, const Quatf &attitude)
	{
		_addDistanceSensorData(distance_sensor, attitude);
	}
	void test_addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw)
	{
		_addObstacleSensorData(obstacle, vehicle_yaw);
	}
	bool test_enterData(int map_index, float sensor_range, float sensor_reading)
	{
		return _enterData(map_index, sensor_range, sensor_reading);
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
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);

	// WHEN: we set the parameter check then apply the setpoint modification
	float value = 10; // try to keep 10m away from obstacles
	param_set(param, &value);
	cp.paramsChanged();

	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: collision prevention should be enabled and limit the speed to zero
	EXPECT_TRUE(cp.is_active());
	EXPECT_FLOAT_EQ(0.f, modified_setpoint.norm());
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithObstacleMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint1(10, 0);
	Vector2f original_setpoint2(-10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param1 = param_handle(px4::params::CP_DIST);
	float value1 = 10; // try to keep 10m distance
	param_set(param1, &value1);
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
	Vector2f modified_setpoint1 = original_setpoint1;
	Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, curr_vel);
	cp.modifySetpoint(modified_setpoint2, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the acceleration setpoint should be negative as its pushing you away from the obstacle, and sideways acceleration should be low
	// case 2: the acceleration setpoint should be lower
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);
	// the obstacles span 0 to 45 deg, so away from them is back and to the left
	const Vector2f obstacles(cosf(math::radians(22.5f)), sinf(math::radians(22.5f)));
	EXPECT_GT(0.f, modified_setpoint1(0)) << modified_setpoint1(0);
	EXPECT_GT(0.f, modified_setpoint1.dot(obstacles)) << modified_setpoint1(1);
	EXPECT_GT(0.f, modified_setpoint2(0))  << original_setpoint2(0);
	EXPECT_GT(0.f, modified_setpoint2.dot(obstacles)) << modified_setpoint2(1);
}

TEST_F(CollisionPreventionTest, testBehaviorOnWithDistanceMessage)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint1(10, 0);
	Vector2f original_setpoint2(-10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
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
	Vector2f modified_setpoint1 = original_setpoint1;
	Vector2f modified_setpoint2 = original_setpoint2;
	cp.modifySetpoint(modified_setpoint1, curr_vel);
	cp.modifySetpoint(modified_setpoint2, curr_vel);
	orb_unadvertise(distance_sensor_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the internal map should know the obstacle
	// case 1: the acceleration setpoint should be negative as its pushing you away from the obstacle, and sideways acceleration should be low
	// case 2: the acceleration setpoint should be lower
	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_FLOAT_EQ(cp.getObstacleMap().min_distance, 100);
	EXPECT_FLOAT_EQ(cp.getObstacleMap().max_distance, 10000);

	EXPECT_GT(0.f, modified_setpoint1(0)) << modified_setpoint1(0);
	EXPECT_LT(fabsf(modified_setpoint1(1)), 0.05f * fabsf(modified_setpoint1(0))) << modified_setpoint1(1);
	EXPECT_GT(0.f, modified_setpoint2(0))  << original_setpoint2(0);
	EXPECT_LT(fabsf(modified_setpoint2(1)), 0.05f * fabsf(modified_setpoint2(0))) << modified_setpoint2(1);
}

TEST_F(CollisionPreventionTest, testPurgeOldData)
{
	// GIVEN: a simple setup condition
	TestTimingCollisionPrevention cp;
	hrt_abstime start_time = hrt_absolute_time();
	mocked_time = start_time;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = start_time;
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 1; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message
	obstacle_distance_s message, message_lost_data;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = start_time;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;
	message_lost_data = message;

	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			message.distances[i] = 10001;
			message_lost_data.distances[i] = UINT16_MAX;

		} else if (i > 15 && i < 18) {
			message.distances[i] = 10001;
			message_lost_data.distances[i] = 10001;

		} else {
			message.distances[i] = UINT16_MAX;
			message_lost_data.distances[i] = UINT16_MAX;
		}
	}

	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	for (int i = 0; i < 10; i++) {
		Vector2f modified_setpoint = original_setpoint;
		cp.modifySetpoint(modified_setpoint, curr_vel);

		mocked_time = mocked_time + 100000; //advance time by 0.1 seconds
		message_lost_data.timestamp = mocked_time;
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message_lost_data);

		// THEN: the stick goes through while the bins are fresh; once they are forgotten the
		// direction is still the pilot's, but 2 m/s is more than the vehicle can stop from within
		// CP_DIST (1 m) of space no sensor reports, so it brakes
		EXPECT_NEAR(modified_setpoint(1), 0.f, 1e-4f);

		if (i < 6) {
			EXPECT_FLOAT_EQ(original_setpoint.norm(), modified_setpoint.norm());

		} else {
			EXPECT_LT(modified_setpoint(0), 0.f);
		}

		// AND: bins not refreshed for half a second are forgotten
		if (i >= 6) {
			EXPECT_EQ(cp.getObstacleMap().distances[0], UINT16_MAX);

		} else {
			EXPECT_EQ(cp.getObstacleMap().distances[0], 10001);
		}
	}

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
}

TEST_F(CollisionPreventionTest, testNoRangeData)
{
	// GIVEN: a simple setup condition
	TestTimingCollisionPrevention cp;
	hrt_abstime start_time = hrt_absolute_time();
	mocked_time = start_time;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = start_time;
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message without any obstacle
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 10000;
	message.angle_offset = 0;
	message.timestamp = start_time;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360 / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		message.distances[i] = 9000;
	}


	// WHEN: we publish the message and set the parameter and then run the setpoint modification
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);

	for (int i = 0; i < 10; i++) {
		Vector2f modified_setpoint = original_setpoint;
		cp.modifySetpoint(modified_setpoint, curr_vel);

		//advance time by 0.1 seconds but no new message comes in
		mocked_time = mocked_time + 100000;

		if (i < 5) {
			// THEN: If the data is new enough, the velocity setpoint should stay the same as the input
			// Note: direction will change slightly due to guidance
			EXPECT_FLOAT_EQ(original_setpoint.norm(), modified_setpoint.norm());

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
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2; // try to keep 2m distance
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
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: setpoint should go into the same direction as the stick input
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(0), modified_setpoint.normalized()(0));
	EXPECT_FLOAT_EQ(original_setpoint.normalized()(1), modified_setpoint.normalized()(1));
}

TEST_F(CollisionPreventionTest, obstacleAheadWhileClosestIsBehind)
{
	// GIVEN: a vehicle facing north at 2 m/s towards an obstacle 2.9 m ahead, with a closer one behind it
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude{};
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 1.5f;
	param_set(param, &value);
	cp.paramsChanged();

	obstacle_distance_s message{};
	message.frame = message.MAV_FRAME_BODY_FRD;
	message.min_distance = 10;
	message.max_distance = 465;
	message.increment = 360.f / bin_count;
	message.timestamp = hrt_absolute_time();

	for (uint i = 0; i < bin_count; i++) {
		message.distances[i] = 466;
	}

	message.distances[0] = 290;
	message.distances[bin_count / 2] = 160;

	// WHEN: the pilot pushes full forward stick
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the obstacle ahead limits the acceleration, although the closest one is behind
	EXPECT_LT(modified_setpoint(0), 1.f);
}

TEST_F(CollisionPreventionTest, noAccelerationTowardsObstacleInsideDistance)
{
	// GIVEN: a vehicle facing north with an obstacle 30 deg right and one behind it on the left, inside CP_DIST
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(0, 0);
	vehicle_attitude_s attitude{};
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 1.5f;
	param_set(param, &value);
	cp.paramsChanged();

	obstacle_distance_s message{};
	message.frame = message.MAV_FRAME_BODY_FRD;
	message.min_distance = 10;
	message.max_distance = 465;
	message.increment = 360.f / bin_count;
	message.timestamp = hrt_absolute_time();

	for (uint i = 0; i < bin_count; i++) {
		message.distances[i] = 466;
	}

	const int ahead_bin = 30 / bin_size;
	const int close_bin = (360 - 110) / bin_size;
	message.distances[ahead_bin] = 200;
	message.distances[close_bin] = 100;

	// WHEN: the pilot pushes the stick forward, which slides it left past the obstacle ahead
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: nothing of the setpoint points towards the obstacle that is already too close
	const float close_angle = math::radians((float)(close_bin * bin_size));
	EXPECT_LE(modified_setpoint.dot(Vector2f(cosf(close_angle), sinf(close_angle))), 1e-5f);
}

TEST_F(CollisionPreventionTest, noAccelerationTowardsEitherWallOfACorridor)
{
	// GIVEN: a vehicle facing north between two walls 1 m away, inside CP_DIST, slightly ahead on each side
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(0, 0);
	vehicle_attitude_s attitude{};
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 1.5f;
	param_set(param, &value);
	cp.paramsChanged();

	obstacle_distance_s message{};
	message.frame = message.MAV_FRAME_BODY_FRD;
	message.min_distance = 10;
	message.max_distance = 465;
	message.increment = 360.f / bin_count;
	message.timestamp = hrt_absolute_time();

	for (uint i = 0; i < bin_count; i++) {
		message.distances[i] = 466;
	}

	const int right_bin = 80 / bin_size;
	const int left_bin = 280 / bin_size;
	message.distances[right_bin] = 100;
	message.distances[left_bin] = 100;

	// WHEN: the pilot pushes the stick forward
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);

	// THEN: the setpoint points towards neither wall
	for (int bin : {right_bin, left_bin}) {
		const float angle = math::radians((float)(bin * bin_size));
		EXPECT_LE(modified_setpoint.dot(Vector2f(cosf(angle), sinf(angle))), 1e-5f);
	}
}

TEST_F(CollisionPreventionTest, pilotMayFlyWhereNoSensorLooks)
{
	// GIVEN: a sensor that covers 45 to 225 deg, with a wall 3 m away there, and CP_DIST 2 m
	TestCollisionPrevention cp;
	Vector2f curr_vel(0, 0);

	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2;
	param_set(param, &value);
	cp.paramsChanged();

	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360.f / distances_array_size;

	for (int i = 0; i < distances_array_size; i++) {
		const float angle = i * message.increment;
		message.distances[i] = (angle > 45.f && angle < 225.f) ? 300 : UINT16_MAX;
	}

	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);

	// WHEN: the stick points into the wall, then north-west where no sensor looks
	message.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	Vector2f towards = {0.f, 10.f};
	cp.modifySetpoint(towards, curr_vel);

	message.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	const Vector2f north_west = {7.f, -7.f};
	Vector2f away = north_west;
	cp.modifySetpoint(away, curr_vel);

	// AND: the wall is at CP_DIST
	for (int i = 0; i < distances_array_size; i++) {
		const float angle = i * message.increment;
		message.distances[i] = (angle > 45.f && angle < 225.f) ? 200 : UINT16_MAX;
	}

	message.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	Vector2f at_distance = {0.f, 10.f};
	cp.modifySetpoint(at_distance, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// THEN: towards the wall only as fast as the vehicle can stop at CP_DIST, and not at all once
	// there; the unobserved direction is the pilot's call, at a speed the vehicle can stop from
	// within CP_DIST
	EXPECT_GT(towards(1), 0.f);
	EXPECT_LT(towards(1), 10.f);
	EXPECT_LE(at_distance(1), 1e-3f);
	EXPECT_GT(away.norm(), 0.f);
	EXPECT_NEAR(away.normalized()(0), north_west.normalized()(0), 1e-4f);
	EXPECT_NEAR(away.normalized()(1), north_west.normalized()(1), 1e-4f);
}

TEST_F(CollisionPreventionTest, pilotMayFlySidewaysIntoUnseenSpaceWithSmallDistance)
{
	// GIVEN: a forward sensor seeing nothing in range, CP_DIST smaller than the margin a direction needs to open
	TestCollisionPrevention cp;
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 0.8f;
	param_set(param, &value);
	cp.paramsChanged();

	obstacle_distance_s message{};
	message.frame = message.MAV_FRAME_BODY_FRD;
	message.min_distance = 10;
	message.max_distance = 465;
	message.increment = 360.f / bin_count;

	for (uint i = 0; i < bin_count; i++) {
		message.distances[i] = (i < 6 || i > bin_count - 6) ? 466 : UINT16_MAX;
	}

	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);

	// WHEN: the pilot pushes the stick to the side, from rest and then at walking pace
	Vector2f from_rest = {0.f, 10.f};
	message.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	cp.modifySetpoint(from_rest, Vector2f(0.f, 0.f));

	Vector2f moving = {0.f, 10.f};
	message.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);
	cp.modifySetpoint(moving, Vector2f(0.f, 0.3f));
	orb_unadvertise(obstacle_distance_pub);

	// THEN: the vehicle moves sideways, slowly
	EXPECT_GT(from_rest(1), 0.f);
	EXPECT_GT(moving(1), 0.f);
	EXPECT_LT(moving(1), 10.f);
}

TEST_F(CollisionPreventionTest, goNoData)
{
	// GIVEN: a simple setup condition with the initial state (no distance data)
	TestCollisionPrevention cp;
	Vector2f curr_vel(2, 0);

	// AND: an obstacle message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.frame = message.MAV_FRAME_GLOBAL; //north aligned
	message.min_distance = 100;
	message.max_distance = 2000;
	int distances_array_size = sizeof(message.distances) / sizeof(message.distances[0]);
	message.increment = 360.f / distances_array_size;

	//fov from 0deg to 20deg
	for (int i = 0; i < distances_array_size; i++) {
		float angle = i * message.increment;

		if (angle > 0.f && angle < 40.f) {
			message.distances[i] = 1000;

		} else {
			message.distances[i] = UINT16_MAX;
		}
	}

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 2; // try to keep 5m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: a setpoint outside the field of view
	Vector2f original_setpoint = {-5, 0};
	Vector2f modified_setpoint = original_setpoint;

	//THEN: without any sensor data the modified setpoint should be zero acceleration
	cp.modifySetpoint(modified_setpoint, curr_vel);
	EXPECT_FLOAT_EQ(modified_setpoint.norm(), 0.f);

	//THEN: As soon as the range data contains any valid number, flying outside the FOV is allowed
	message.timestamp = hrt_absolute_time();
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &message);

	modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);
	EXPECT_FLOAT_EQ(modified_setpoint.norm(), original_setpoint.norm());
	orb_unadvertise(obstacle_distance_pub);
}

TEST_F(CollisionPreventionTest, jerkLimit)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);

	// AND: distance set to 5m
	param_t param = param_handle(px4::params::CP_DIST);
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
	Vector2f modified_setpoint_default_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_default_jerk, curr_vel);
	orb_unadvertise(obstacle_distance_pub);

	// AND: we now set max jerk to 0.1
	param = param_handle(px4::params::MPC_JERK_MAX);
	value = 0.1; // 0.1 maximum jerk
	param_set(param, &value);
	cp.paramsChanged();

	// WHEN: we run the setpoint modification again
	Vector2f modified_setpoint_limited_jerk = original_setpoint;
	cp.modifySetpoint(modified_setpoint_limited_jerk, curr_vel);

	// THEN: the new setpoint should be much higher than the one with default jerk, as the rate of change in acceleration is more limmited
	EXPECT_GT(modified_setpoint_limited_jerk.norm(), modified_setpoint_default_jerk.norm());

}
TEST_F(CollisionPreventionTest, addOutOfRangeDistanceSensorData)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	// Distance is out of Range
	distance_sensor.current_distance = -1.f;
	distance_sensor.signal_quality = 0;

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 0) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], distance_sensor.max_distance * 100.f);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}
	}
}

TEST_F(CollisionPreventionTest, addDistanceSensorDataNarrow)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(0.1 * bin_size);

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	// WHEN the sensor has a very narrow field of view
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 0) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}
}
TEST_F(CollisionPreventionTest, addDistanceSensorDataSlightlyLarger)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(1.1 * bin_size);

	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	// WHEN the sensor has a very narrow field of view
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the the bins corresponding to -5°, 0° and 5° should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i == 71 || i <= 1) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}
}

TEST_F(CollisionPreventionTest, addDistanceSensorData)
{
	// GIVEN: a vehicle attitude and a distance sensor message
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.current_distance = 5.f;

	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	uint32_t distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (uint32_t i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
	}

	//WHEN: we add distance sensor data to the right
	distance_sensor.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
	distance_sensor.h_fov = math::radians(19.99f);
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	uint fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start = (90 - fov) / bin_size;
	uint end = (90 + fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}

	//WHEN: we add additionally distance sensor data to the left
	distance_sensor.orientation = distance_sensor_s::ROTATION_LEFT_FACING;
	distance_sensor.h_fov = math::radians(50.f);
	distance_sensor.current_distance = 8.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start2 = (270 - fov) / bin_size;
	uint end2 = (270 + fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else if (i >= start2 && i <= end2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}

	//WHEN: we add additionally distance sensor data to the front
	distance_sensor.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	distance_sensor.h_fov = math::radians(10.1f);
	distance_sensor.current_distance = 3.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);
	fov = round(distance_sensor.h_fov * M_RAD_TO_DEG_F / 2);
	uint start3 = (360 - fov) / bin_size;
	uint end3 = (fov) / bin_size;

	//THEN: the correct bins in the map should be filled
	for (uint32_t i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500)  << i;

		} else if (i >= start2 && i <= end2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 800) << i;

		} else if (i >= start3 || i <= end3) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 300) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}
	}


}

TEST_F(CollisionPreventionTest, addObstacleSensorData_attitude)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 10-30 deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	int start = 2;
	int end = 6;

	for (int i = start; i <= end ; i++) {
		obstacle_msg.distances[i] = 500;
	}



	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add distance sensor data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}


	//WHEN: we add obstacle distance sensor data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, M_PI_2);

	//THEN: the correct bins in the map should be filled
	int offset =  bin_count - 90 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle distance sensor data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, -M_PI_4);

	//THEN: the correct bins in the map should be filled
	offset =  45 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle distance sensor data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, M_PI);

	//THEN: the correct bins in the map should be filled
	offset =  180 / bin_size;

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= offset + start && i <= offset + end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500);

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, addObstacleSensorData_offset_bodyframe)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD; // Body Frame
	obstacle_msg.increment = 6.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 363°-39° deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i <= 6 ; i++) { // 36° at 6° increment
		obstacle_msg.distances[i] = 500;
	}

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the the bins from 0 to 40 in map should be filled, which correspond to the angles from -2.5° to 42.5°
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 0 && i <= 8) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add the same obstacle distance sensor data with an angle offset of 30.5°
	obstacle_msg.angle_offset = 30.5f;
	// This then means our obstacle is between 27.5° and 69.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 30° to 70° in map should be filled, which correspond to the angles from 27.5° to 72.5°
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 6 && i <= 14) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we increase the offset to -30.5°
	obstacle_msg.angle_offset = -30.5f;
	// This then means our obstacle is between 326.5° and 8.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 325° to 10° in map should be filled, which correspond to the angles from 322.5° to 12.5°

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 65 || i <= 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, addObstacleSensorData_bodyframe)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_BODY_FRD; //north aligned
	obstacle_msg.increment = 5.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 10-30 deg body frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));
	int start = 2;
	int end = 6;

	for (int i = start; i <= end ; i++) {
		obstacle_msg.distances[i] = 500;
	}


	//THEN: at initialization the internal obstacle map should only contain UINT16_MAX
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX);
	}

	//WHEN: we add obstacle data while vehicle has zero yaw
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the correct bins in the map should be filled

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 90deg to the right
	cp.test_addObstacleSensorData(obstacle_msg, M_PI_2);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 45deg to the left
	cp.test_addObstacleSensorData(obstacle_msg, -M_PI_4);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add obstacle data while vehicle yaw 180deg
	cp.test_addObstacleSensorData(obstacle_msg, M_PI);

	//THEN: the correct bins in the map should be filled
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= start && i <= end) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

}


TEST_F(CollisionPreventionTest, addObstacleSensorData_resolution_offset)
{
	// GIVEN: a vehicle attitude and obstacle distance message
	TestCollisionPrevention cp;
	obstacle_distance_s obstacle_msg {};
	obstacle_msg.frame = obstacle_msg.MAV_FRAME_GLOBAL; //north aligned
	obstacle_msg.increment = 6.f;
	obstacle_msg.min_distance = 20;
	obstacle_msg.max_distance = 2000;
	obstacle_msg.angle_offset = 0.f;

	//obstacle at 363°-39° deg world frame, distance 5 meters
	memset(&obstacle_msg.distances[0], UINT16_MAX, sizeof(obstacle_msg.distances));

	for (int i = 0; i <= 6 ; i++) { // 36° at 6° increment
		obstacle_msg.distances[i] = 500;
	}

	//WHEN: we add distance sensor data
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the the bins from 0 to 40 in map should be filled, which correspond to the angles from -2.5° to 42.5°
	int distances_array_size = sizeof(cp.getObstacleMap().distances) / sizeof(cp.getObstacleMap().distances[0]);

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 0 && i <= 8) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we add the same obstacle distance sensor data with an angle offset of 30.5°
	obstacle_msg.angle_offset = 30.5f;
	// This then means our obstacle is between 27.5° and 69.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 30° to 70° in map should be filled, which correspond to the angles from 27.5° to 72.5°
	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 6 && i <= 14) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}

	//WHEN: we increase the offset to -30.5°
	obstacle_msg.angle_offset = -30.5f;
	// This then means our obstacle is between 326.5° and 8.5°
	cp.test_addObstacleSensorData(obstacle_msg, 0.f);

	//THEN: the bins from 325° to 10° in map should be filled, which correspond to the angles from 322.5° to 12.5°

	for (int i = 0; i < distances_array_size; i++) {
		if (i >= 65 || i <= 2) {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], 500) << i;

		} else {
			EXPECT_FLOAT_EQ(cp.getObstacleMap().distances[i], UINT16_MAX) << i;
		}

		//reset array to UINT16_MAX
		cp.getObstacleMap().distances[i] = UINT16_MAX;
	}
}

TEST_F(CollisionPreventionTest, overlappingSensors)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Vector2f original_setpoint(10, 0);
	Vector2f curr_vel(2, 0);
	vehicle_attitude_s attitude;
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.0f;
	attitude.q[1] = 0.0f;
	attitude.q[2] = 0.0f;
	attitude.q[3] = 0.0f;

	// AND: a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);
	float value = 10; // try to keep 10m distance
	param_set(param, &value);
	cp.paramsChanged();

	// AND: an obstacle message for a short range and a long range sensor
	obstacle_distance_s short_range_msg, short_range_msg_no_obstacle, long_range_msg;
	memset(&short_range_msg, 0xDEAD, sizeof(short_range_msg));
	short_range_msg.frame = short_range_msg.MAV_FRAME_GLOBAL; //north aligned
	short_range_msg.angle_offset = 0;
	short_range_msg.timestamp = hrt_absolute_time();
	int distances_array_size = sizeof(short_range_msg.distances) / sizeof(short_range_msg.distances[0]);
	short_range_msg.increment = 360 / distances_array_size;
	long_range_msg = short_range_msg;
	long_range_msg.min_distance = 100;
	long_range_msg.max_distance = 1000;
	short_range_msg.min_distance = 20;
	short_range_msg.max_distance = 200;
	short_range_msg_no_obstacle = short_range_msg;


	for (int i = 0; i < distances_array_size; i++) {
		if (i < 10) {
			short_range_msg_no_obstacle.distances[i] = 201;
			short_range_msg.distances[i] = 150;
			long_range_msg.distances[i] = 500;

		} else {
			short_range_msg_no_obstacle.distances[i] = UINT16_MAX;
			short_range_msg.distances[i] = UINT16_MAX;
			long_range_msg.distances[i] = UINT16_MAX;
		}
	}


	// CASE 1
	//WHEN: we publish the long range sensor message
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &long_range_msg);
	orb_advert_t vehicle_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	orb_publish(ORB_ID(vehicle_attitude), vehicle_attitude_pub, &attitude);
	Vector2f modified_setpoint = original_setpoint;
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the long range measurement
	EXPECT_EQ(500, cp.getObstacleMap().distances[2]);

	// CASE 2
	// WHEN: we publish the short range message followed by a long range message
	short_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &short_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);
	long_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the short range measurement
	EXPECT_EQ(150, cp.getObstacleMap().distances[2]);

	// CASE 3
	// WHEN: we publish the short range message with values out of range followed by a long range message
	short_range_msg_no_obstacle.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &short_range_msg_no_obstacle);
	cp.modifySetpoint(modified_setpoint, curr_vel);
	long_range_msg.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &long_range_msg);
	cp.modifySetpoint(modified_setpoint, curr_vel);

	// THEN: the internal map data should contain the short range measurement
	EXPECT_EQ(500, cp.getObstacleMap().distances[2]);

	orb_unadvertise(obstacle_distance_pub);
	orb_unadvertise(vehicle_attitude_pub);
}

TEST_F(CollisionPreventionTest, enterData)
{
	// GIVEN: a simple setup condition
	TestCollisionPrevention cp;
	Quaternion<float> vehicle_attitude(1, 0, 0, 0); //unit transform

	//THEN: just after initialization all bins are at UINT16_MAX and any data should be accepted
	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range

	//WHEN: we add distance sensor data to the right with a valid reading
	distance_sensor_s distance_sensor {};
	distance_sensor.min_distance = 0.2f;
	distance_sensor.max_distance = 20.f;
	distance_sensor.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
	distance_sensor.h_fov = math::radians(19.99f);
	distance_sensor.current_distance = 5.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the internal map should contain the distance sensor readings
	for (int i = 16; i < 20; i++) {
		EXPECT_EQ(500, cp.getObstacleMap().distances[i]) << i;
	}

	//THEN: bins 8 & 9 contain valid readings
	// a valid reading should only be accepted from sensors with shorter or equal range
	// a out of range reading should only be accepted from sensors with the same range

	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_FALSE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range

	//WHEN: we add distance sensor data to the right with an out of range reading
	distance_sensor.current_distance = 21.f;
	cp.test_addDistanceSensorData(distance_sensor, vehicle_attitude);

	//THEN: the internal map should contain the distance sensor readings
	for (int i = 16; i < 20; i++) {
		EXPECT_EQ(2000, cp.getObstacleMap().distances[i]) << i;
	}

	//THEN: bins 8 & 9 contain readings out of range
	// a reading in range will be accepted in any case
	// out of range readings will only be accepted from sensors with bigger or equal range

	EXPECT_TRUE(cp.test_enterData(16, 2.f, 1.5f)); //shorter range, reading in range
	EXPECT_FALSE(cp.test_enterData(16, 2.f, 3.f)); //shorter range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 1.5f)); //same range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 20.f, 21.f)); //same range, reading out of range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 1.5f)); //longer range, reading in range
	EXPECT_TRUE(cp.test_enterData(16, 30.f, 31.f)); //longer range, reading out of range
}

// clear all round to the map's 4.65 m, so only obstacle_clearance limits anything
static obstacle_distance_s clearSurroundings()
{
	obstacle_distance_s message{};
	message.frame = message.MAV_FRAME_BODY_FRD;
	message.min_distance = 10;
	message.max_distance = 465;
	message.increment = 360.f / bin_count;

	for (uint i = 0; i < bin_count; i++) {
		message.distances[i] = 466;
	}

	return message;
}

// swept up and down to contact at up and down [m], all observed, and nowhere else
static obstacle_clearance_s clearanceAboveAndBelow(float up, float down)
{
	obstacle_clearance_s clearance{};
	clearance.body_radius = 0.3f;
	clearance.max_distance = 4.65f;

	for (int i = 0; i < obstacle_clearance_s::SWEEP_COUNT; i++) {
		clearance.contact[i] = NAN;
		clearance.observed[i] = NAN;
	}

	clearance.direction_down[obstacle_clearance_s::SWEEP_UP] = -1.f;
	clearance.contact[obstacle_clearance_s::SWEEP_UP] = up;
	clearance.observed[obstacle_clearance_s::SWEEP_UP] = PX4_ISFINITE(up) ? up : 2.25f;
	clearance.contact_face[obstacle_clearance_s::SWEEP_UP] = obstacle_clearance_s::FACE_TOP;
	clearance.direction_down[obstacle_clearance_s::SWEEP_DOWN] = 1.f;
	clearance.contact[obstacle_clearance_s::SWEEP_DOWN] = down;
	clearance.observed[obstacle_clearance_s::SWEEP_DOWN] = PX4_ISFINITE(down) ? down : 2.25f;
	clearance.contact_face[obstacle_clearance_s::SWEEP_DOWN] = obstacle_clearance_s::FACE_BOTTOM;
	return clearance;
}

class ClearanceTest : public CollisionPreventionTest
{
public:
	void SetUp() override
	{
		CollisionPreventionTest::SetUp();
		float value = 1.5f;
		param_set(param_handle(px4::params::CP_DIST), &value);
		cp.paramsChanged();
		obstacles = clearSurroundings();
		obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &obstacles);
		obstacle_clearance_s none{};
		clearance_pub = orb_advertise(ORB_ID(obstacle_clearance), &none);
	}

	void TearDown() override
	{
		orb_unadvertise(obstacle_distance_pub);
		orb_unadvertise(clearance_pub);
	}

	// one Collision Prevention cycle with fresh data, returns the limited acceleration
	Vector2f cycle(obstacle_clearance_s clearance, const Vector2f &acceleration, const Vector2f &velocity)
	{
		obstacles.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &obstacles);
		clearance.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(obstacle_clearance), clearance_pub, &clearance);
		Vector2f limited = acceleration;
		cp.is_active();
		cp.modifySetpoint(limited, velocity);
		return limited;
	}

	TestCollisionPrevention cp;
	obstacle_distance_s obstacles{};
	orb_advert_t obstacle_distance_pub{nullptr};
	orb_advert_t clearance_pub{nullptr};
};

TEST_F(ClearanceTest, climbSlowsToStopShortOfACeiling)
{
	float up = NAN;
	float down = NAN;

	// WHEN: a ceiling 2 m over the top of the vehicle, then 0.5 m, as far as CP_DIST_V
	cycle(clearanceAboveAndBelow(2.f, INFINITY), Vector2f(), Vector2f());
	ASSERT_TRUE(cp.verticalSpeedLimits(up, down));
	const float far = up;
	cycle(clearanceAboveAndBelow(0.5f, INFINITY), Vector2f(), Vector2f());
	ASSERT_TRUE(cp.verticalSpeedLimits(up, down));

	// THEN: the climb slows down to nothing
	EXPECT_GT(far, 1.f);
	EXPECT_FLOAT_EQ(up, 0.f);
}

TEST_F(ClearanceTest, descentNeverSlowsBelowLandingSpeed)
{
	float up = NAN;
	float down = NAN;
	float land_speed = NAN;
	param_get(param_handle(px4::params::MPC_LAND_SPEED), &land_speed);

	// WHEN: the ground is 0.1 m under the bottom of the vehicle
	cycle(clearanceAboveAndBelow(INFINITY, 0.1f), Vector2f(), Vector2f());
	ASSERT_TRUE(cp.verticalSpeedLimits(up, down));

	// THEN: the vehicle can still come down to land
	EXPECT_FLOAT_EQ(down, land_speed);
}

TEST_F(ClearanceTest, unseenSpaceAboveCapsTheClimb)
{
	// WHEN: nothing over the vehicle was ever observed
	obstacle_clearance_s clearance = clearanceAboveAndBelow(INFINITY, INFINITY);
	clearance.observed[obstacle_clearance_s::SWEEP_UP] = 0.f;
	cycle(clearance, Vector2f(), Vector2f());

	// THEN: it climbs, only as fast as it can stop within CP_DIST_V
	float up = NAN;
	float down = NAN;
	ASSERT_TRUE(cp.verticalSpeedLimits(up, down));
	EXPECT_GT(up, 0.2f);
	EXPECT_LT(up, 2.5f);
}

TEST_F(ClearanceTest, somethingInThePathBrakesTheVehicleTowardsIt)
{
	// WHEN: flying north at 0.5 m/s, the body would touch something 0.5 m along its setpoint, though
	// no sector holds it
	obstacle_clearance_s clearance = clearanceAboveAndBelow(INFINITY, INFINITY);
	clearance.direction_north[obstacle_clearance_s::SWEEP_SETPOINT] = 1.f;
	clearance.contact[obstacle_clearance_s::SWEEP_SETPOINT] = 0.5f;
	clearance.observed[obstacle_clearance_s::SWEEP_SETPOINT] = 0.5f;
	clearance.contact_face[obstacle_clearance_s::SWEEP_SETPOINT] = obstacle_clearance_s::FACE_SIDE;

	const Vector2f free_run = cycle(clearanceAboveAndBelow(INFINITY, INFINITY), Vector2f(10.f, 0.f), Vector2f(0.5f, 0.f));
	const Vector2f limited = cycle(clearance, Vector2f(10.f, 0.f), Vector2f(0.5f, 0.f));

	// THEN: the stick accelerates without it, and brakes with it
	EXPECT_GT(free_run(0), 3.f);
	EXPECT_LT(limited(0), 0.f);
}

TEST_F(ClearanceTest, aCeilingAheadStopsTheClimbButNotTheFlight)
{
	// WHEN: climbing forward at 45 degrees, the top of the body would reach a ceiling 0.7 m along
	// the way, less than CP_DIST_V above it
	const Vector2f free_run = cycle(clearanceAboveAndBelow(INFINITY, INFINITY), Vector2f(10.f, 0.f), Vector2f(1.f, 0.f));
	obstacle_clearance_s clearance = clearanceAboveAndBelow(INFINITY, INFINITY);
	clearance.direction_north[obstacle_clearance_s::SWEEP_SETPOINT] = M_SQRT1_2_F;
	clearance.direction_down[obstacle_clearance_s::SWEEP_SETPOINT] = -M_SQRT1_2_F;
	clearance.contact[obstacle_clearance_s::SWEEP_SETPOINT] = 0.7f;
	clearance.observed[obstacle_clearance_s::SWEEP_SETPOINT] = 0.7f;
	clearance.contact_face[obstacle_clearance_s::SWEEP_SETPOINT] = obstacle_clearance_s::FACE_TOP;
	const Vector2f limited = cycle(clearance, Vector2f(10.f, 0.f), Vector2f(1.f, 0.f));

	// THEN: the climb stops, forward flight goes on under it
	float up = NAN;
	float down = NAN;
	ASSERT_TRUE(cp.verticalSpeedLimits(up, down));
	EXPECT_FLOAT_EQ(up, 0.f);
	EXPECT_FLOAT_EQ(limited(0), free_run(0));
}

TEST_F(ClearanceTest, staleClearanceLimitsNothing)
{
	// WHEN: the last clearance is a second old
	obstacle_clearance_s clearance = clearanceAboveAndBelow(0.1f, 0.1f);
	clearance.timestamp = hrt_absolute_time() - 1_s;
	orb_publish(ORB_ID(obstacle_clearance), clearance_pub, &clearance);
	obstacles.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(obstacle_distance), obstacle_distance_pub, &obstacles);
	Vector2f acceleration(0.f, 0.f);
	cp.is_active();
	cp.modifySetpoint(acceleration, Vector2f());

	// THEN: no vertical limits
	float up = NAN;
	float down = NAN;
	EXPECT_FALSE(cp.verticalSpeedLimits(up, down));
}
