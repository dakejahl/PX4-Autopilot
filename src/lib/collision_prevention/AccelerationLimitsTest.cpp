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

#include "AccelerationLimits.hpp"

#include <math.h>

using namespace collision_prevention;
using matrix::Vector2f;

static constexpr float kRange = 4.65f;

static PolarObstacles clearField()
{
	PolarObstacles obstacles{};

	for (int k = 0; k < kSectors; k++) {
		obstacles.distance[k] = INFINITY;
		obstacles.range[k] = kRange;
	}

	return obstacles;
}

static PolarObstacles unseenField()
{
	PolarObstacles obstacles = clearField();

	for (int k = 0; k < kSectors; k++) {
		obstacles.distance[k] = NAN;
	}

	return obstacles;
}

// a straight wall across the sectors facing it, at distance along the sector at angle
static void addWall(PolarObstacles &obstacles, float angle, float distance)
{
	for (int k = 0; k < kSectors; k++) {
		const float off = sectorDirection(k).dot(Vector2f(cosf(angle), sinf(angle)));

		if (off > 0.1f) {
			obstacles.distance[k] = fminf(obstacles.distance[k], distance / off);
		}
	}
}

static LimitConfig config()
{
	LimitConfig c{};
	c.distance = 1.f;
	c.max_acceleration = 3.f;
	c.max_jerk = 10.f;
	c.gain = 2.f;
	c.delay = 0.f;
	c.push_time = 1.f;
	return c;
}

static float degrees(float deg) { return deg * M_PI_F / 180.f; }

static LimitWorkspace workspace;

TEST(AccelerationLimits, FarObstacleLeavesTheStickAlone)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[0] = 4.f;
	const Vector2f stick(3.f, 0.f);
	const Vector2f limited = limitAcceleration(stick, Vector2f(), Vector2f(), obstacles, config(), workspace);
	EXPECT_NEAR(limited(0), 3.f, 1e-4f);
	EXPECT_NEAR(limited(1), 0.f, 1e-4f);
}

TEST(AccelerationLimits, BrakesTowardsACloseObstacle)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[0] = 2.f;
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(3.f, 0.f), Vector2f(3.f, 0.f), obstacles,
				 config(), workspace);
	EXPECT_LT(limited(0), 0.f);
}

TEST(AccelerationLimits, LaggingVehicleBrakesWhenTheSetpointHasStopped)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[0] = 1.5f;
	// the setpoint has stopped, the vehicle still flies towards the obstacle
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(), Vector2f(2.f, 0.f), obstacles, config(), workspace);
	EXPECT_LT(limited(0), 0.f);
}

TEST(AccelerationLimits, StraightIntoAnObstacleStopsWithoutTurning)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[0] = 1.f;
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(), Vector2f(), obstacles, config(), workspace);
	EXPECT_LE(limited(0), 1e-3f);
	EXPECT_NEAR(limited(1), 0.f, 1e-3f);
}

TEST(AccelerationLimits, PastAWallSlidesAlongIt)
{
	// a wall to the north at the distance kept, the stick north-east
	PolarObstacles obstacles = clearField();
	addWall(obstacles, 0.f, 1.f);
	const Vector2f stick = Vector2f(1.f, 1.f).normalized() * 3.f;
	const Vector2f limited = limitAcceleration(stick, Vector2f(), Vector2f(), obstacles, config(), workspace);

	// no closer to the wall, on along it, and never against the command
	EXPECT_LE(limited(0), 1e-3f);
	EXPECT_GT(limited(1), 0.5f);
	EXPECT_LE(limited(1), stick(1) + 1e-3f);
}

TEST(AccelerationLimits, NoSlideIntoUnseenSpace)
{
	// the same wall, with nothing seen east of it
	PolarObstacles obstacles = clearField();
	addWall(obstacles, 0.f, 1.f);

	for (int k = sectorOf(degrees(60.f)); k <= sectorOf(degrees(120.f)); k++) {
		obstacles.distance[k] = NAN;
	}

	const Vector2f stick = Vector2f(1.f, 1.f).normalized() * 3.f;
	const Vector2f limited = limitAcceleration(stick, Vector2f(), Vector2f(), obstacles, config(), workspace);

	// stops at the wall rather than sliding along it into the unseen
	EXPECT_LE(limited(0), 1e-3f);
	EXPECT_LE(limited(1), 1e-2f);

	// though the pilot may command it there
	EXPECT_GT(limitAcceleration(Vector2f(0.f, 3.f), Vector2f(), Vector2f(), obstacles, config(), workspace)(1), 1.f);
}

TEST(AccelerationLimits, NothingIsTurnedTowardsWithoutACommand)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[sectorOf(degrees(30.f))] = 1.5f;
	// no stick, hovering
	const Vector2f limited = limitAcceleration(Vector2f(), Vector2f(), Vector2f(), obstacles, config(), workspace);
	EXPECT_NEAR(limited.norm(), 0.f, 1e-4f);
}

TEST(AccelerationLimits, PilotMayFlyWhereNoSensorLooksButSlowly)
{
	const PolarObstacles obstacles = unseenField();
	const Vector2f stick(0.f, 3.f);
	// from rest it accelerates
	EXPECT_GT(limitAcceleration(stick, Vector2f(), Vector2f(), obstacles, config(), workspace)(1), 1.f);
	// faster than it can stop within the distance kept, it brakes
	EXPECT_LT(limitAcceleration(stick, Vector2f(0.f, 4.f), Vector2f(0.f, 4.f), obstacles, config(), workspace)(1), 0.f);
}

TEST(AccelerationLimits, BacksAwayFromAnObstacleInsideTheDistance)
{
	PolarObstacles obstacles = clearField();
	obstacles.distance[0] = 0.5f;
	const Vector2f limited = limitAcceleration(Vector2f(), Vector2f(), Vector2f(), obstacles, config(), workspace);
	EXPECT_LT(limited(0), -0.1f);
	EXPECT_NEAR(limited(1), 0.f, 1e-3f);
}

TEST(AccelerationLimits, CorridorNarrowerThanTheDistanceStillLetsTheVehicleThrough)
{
	// walls east and west, each 0.8 m away with 1 m kept
	PolarObstacles obstacles = clearField();
	addWall(obstacles, degrees(90.f), 0.8f);
	addWall(obstacles, degrees(-90.f), 0.8f);
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(), Vector2f(), obstacles, config(), workspace);

	// on along the corridor, pushed towards neither wall
	EXPECT_GT(limited(0), 0.5f);
	EXPECT_NEAR(limited(1), 0.f, 1e-2f);
}

TEST(AccelerationLimits, ObservedClearAheadIsNotSlowedByUnseenSides)
{
	// a forward sensor's view: clear ahead within 30 degrees, nothing seen beyond
	PolarObstacles obstacles = unseenField();

	for (int k = -6; k <= 6; k++) {
		obstacles.distance[(k + kSectors) % kSectors] = INFINITY;
	}

	// at 3 m/s ahead the stick still accelerates
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(3.f, 0.f), Vector2f(3.f, 0.f), obstacles,
				 config(), workspace);
	EXPECT_GT(limited(0), 0.f);
}

TEST(AccelerationLimits, PathLimitBrakesAlongItsDirection)
{
	const PolarObstacles obstacles = clearField();
	const PathLimit path{Vector2f(1.f, 0.f), 0.f};
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(1.f, 0.f), Vector2f(1.f, 0.f), obstacles,
				 config(), workspace, &path, 1);
	EXPECT_LT(limited(0), 0.f);
}

TEST(AccelerationLimits, CorridorStillBrakesWhenSetpointAndVehicleDriftApart)
{
	// walls either side at the distance kept, a wall ahead, flying at it with the vehicle drifting
	// slightly across while the setpoint does not
	PolarObstacles obstacles = clearField();
	addWall(obstacles, degrees(90.f), 1.f);
	addWall(obstacles, degrees(-90.f), 1.f);
	addWall(obstacles, 0.f, 2.5f);
	LimitConfig c = config();
	c.gain = 1.8f;
	c.delay = 0.4f;
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(2.f, 0.f), Vector2f(2.f, 0.05f), obstacles, c,
				 workspace);
	EXPECT_LT(limited(0), -0.5f);
}

TEST(AccelerationLimits, BrakingTowardsUnseenSpaceIsNoSlide)
{
	// a forward sensor's view, a wall ahead, the vehicle lagging its setpoint towards it
	PolarObstacles obstacles = unseenField();

	for (int k = -6; k <= 6; k++) {
		obstacles.distance[(k + kSectors) % kSectors] = INFINITY;
	}

	addWall(obstacles, 0.f, 1.6f);
	const Vector2f limited = limitAcceleration(Vector2f(3.f, 0.f), Vector2f(0.5f, 0.f), Vector2f(1.5f, 0.f), obstacles,
				 config(), workspace);
	EXPECT_LT(limited(0), -0.5f);
}

TEST(AccelerationLimits, SeenClearIsNeverSlowerThanUnseen)
{
	// a sensor that sees only 2 m, with 1.5 m kept
	PolarObstacles seen = clearField();
	PolarObstacles unseen = unseenField();

	for (int k = 0; k < kSectors; k++) {
		seen.range[k] = 2.f;
	}

	LimitConfig c = config();
	c.distance = 1.5f;
	const Vector2f stick(3.f, 0.f);
	const Vector2f velocity(0.9f, 0.f);
	EXPECT_GE(limitAcceleration(stick, velocity, velocity, seen, c, workspace)(0),
		  limitAcceleration(stick, velocity, velocity, unseen, c, workspace)(0) - 1e-4f);
}
