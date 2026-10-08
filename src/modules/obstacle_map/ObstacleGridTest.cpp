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

#include "ObstacleGrid.hpp"

#include <math.h>

using namespace obstacle_map;
using State = ObstacleGrid::State;

static constexpr float kVoxel = 0.15f;
static constexpr int kBins = 72;
static constexpr float kPi = 3.14159265f;
// mid-voxel, so the wall does not sit on a voxel boundary
static constexpr float kWall = 3.075f;
static constexpr float kRadius = 0.3f;
static constexpr float kHalfHeight = 0.3f;

class ObstacleGridTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		ASSERT_TRUE(grid.allocate(64, 32));
		grid.setVoxelSize(kVoxel);
		grid.recenter(origin);
	}

	// a fan of rays from origin hitting a wall across north at wall_north
	void insertWall(float wall_north, int frames)
	{
		for (int frame = 0; frame < frames; frame++) {
			for (int i = -10; i <= 10; i++) {
				const float east = i * 0.1f;
				const float dn = wall_north - origin[0];
				const float length = sqrtf(dn * dn + east * east);
				const float direction[3] {dn / length, east / length, 0.f};
				grid.insertRay(origin, direction, length, true);
			}
		}
	}

	State stateAt(float north, float east, float down) const
	{
		return grid.state(grid.voxelIndex(north), grid.voxelIndex(east), grid.voxelIndex(down));
	}

	// two returns from the vehicle at the centre of each voxel of a horizontal slab
	void insertSlab(float north_min, float north_max, float east_min, float east_max, float down)
	{
		for (int frame = 0; frame < 2; frame++) {
			for (float north = north_min; north <= north_max; north += kVoxel) {
				for (float east = east_min; east <= east_max; east += kVoxel) {
					const float target[3] {centre(north), centre(east), centre(down)};
					const float d[3] {target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
					const float length = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
					const float direction[3] {d[0] / length, d[1] / length, d[2] / length};
					grid.insertRay(origin, direction, length, true);
				}
			}
		}
	}

	float centre(float position) const { return (grid.voxelIndex(position) + 0.5f) * kVoxel; }

	Band band(float margin = 0.f) const { return bandAround(grid, origin, kRadius, kHalfHeight, margin); }

	Sweep sweep(float north, float east, float down) const
	{
		const float length = sqrtf(north * north + east * east + down * down);
		const float direction[3] {north / length, east / length, down / length};
		return sweepBody(grid, origin, direction, kRadius, band(), 4.65f);
	}

	ObstacleGrid grid;
	float origin[3] {0.07f, 0.07f, -2.07f};
};

TEST_F(ObstacleGridTest, RejectsSizesThatAreNotPowersOfTwo)
{
	ObstacleGrid other;
	EXPECT_FALSE(other.allocate(48, 32));
	EXPECT_FALSE(other.allocate(64, 2));
	EXPECT_TRUE(other.allocate(16, 8));
}

TEST_F(ObstacleGridTest, StartsUnknown)
{
	EXPECT_EQ(stateAt(1.f, 0.f, -2.f), State::Unknown);
	EXPECT_EQ(stateAt(-4.f, 3.f, -1.f), State::Unknown);
}

TEST_F(ObstacleGridTest, WallIsOccupiedAfterTwoFramesAndSpaceBeforeItIsFree)
{
	insertWall(kWall, 1);
	EXPECT_EQ(stateAt(kWall, 0.f, origin[2]), State::Unknown);

	insertWall(kWall, 1);
	EXPECT_EQ(stateAt(kWall, 0.f, origin[2]), State::Occupied);
	EXPECT_EQ(stateAt(1.5f, 0.f, origin[2]), State::Free);
	// behind the wall was never observed
	EXPECT_EQ(stateAt(kWall + 0.6f, 0.f, origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, SingleGlitchStaysBelowOccupied)
{
	const float direction[3] {1.f, 0.f, 0.f};
	grid.insertRay(origin, direction, 2.f, true);
	EXPECT_EQ(stateAt(origin[0] + 2.f, origin[1], origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, ObstacleThatLeavesClearsAfterFiveMisses)
{
	const float direction[3] {1.f, 0.f, 0.f};

	// saturate a voxel at 2 m
	for (int i = 0; i < 10; i++) {
		grid.insertRay(origin, direction, 2.f, true);
	}

	EXPECT_EQ(stateAt(origin[0] + 2.f, origin[1], origin[2]), State::Occupied);

	// now rays pass through it to a wall at 4 m
	for (int i = 0; i < 4; i++) {
		grid.insertRay(origin, direction, 4.f, true);
	}

	EXPECT_EQ(stateAt(origin[0] + 2.f, origin[1], origin[2]), State::Occupied);
	grid.insertRay(origin, direction, 4.f, true);
	EXPECT_NE(stateAt(origin[0] + 2.f, origin[1], origin[2]), State::Occupied);
}

TEST_F(ObstacleGridTest, NoReturnClearsToRange)
{
	const float direction[3] {1.f, 0.f, 0.f};

	for (int i = 0; i < 2; i++) {
		grid.insertRay(origin, direction, 3.f, false);
	}

	EXPECT_EQ(stateAt(origin[0] + 2.9f, origin[1], origin[2]), State::Free);
	EXPECT_EQ(stateAt(origin[0] + 3.2f, origin[1], origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, VoxelNextToTheEndpointKeepsItsValue)
{
	const float direction[3] {1.f, 0.f, 0.f};

	for (int i = 0; i < 4; i++) {
		grid.insertRay(origin, direction, 2.f, true);
	}

	// a grazing ray ending in the voxel just past it does not take a miss from it
	const int before = grid.value(grid.voxelIndex(origin[0] + 2.f), grid.voxelIndex(origin[1]), grid.voxelIndex(origin[2]));
	grid.insertRay(origin, direction, 2.f + kVoxel, true);
	EXPECT_EQ(grid.value(grid.voxelIndex(origin[0] + 2.f), grid.voxelIndex(origin[1]), grid.voxelIndex(origin[2])), before);
}

TEST_F(ObstacleGridTest, RaysStopAtTheWindowEdge)
{
	const float direction[3] {1.f, 0.f, 0.f};
	// 64 voxels at 0.15 m reach 4.8 m either side
	grid.insertRay(origin, direction, 9.f, true);
	grid.insertRay(origin, direction, 9.f, true);
	EXPECT_EQ(stateAt(origin[0] + 4.f, origin[1], origin[2]), State::Free);
	EXPECT_EQ(stateAt(origin[0] + 9.f, origin[1], origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, ScrollingKeepsTheMapAndForgetsWhatLeaves)
{
	insertWall(kWall, 2);
	ASSERT_EQ(stateAt(kWall, 0.f, origin[2]), State::Occupied);

	// move 2 m south, the wall is 5 m away and outside the 4.8 m half window
	const float south[3] {origin[0] - 2.f, origin[1], origin[2]};
	grid.recenter(south);
	EXPECT_EQ(stateAt(kWall, 0.f, origin[2]), State::Unknown);
	EXPECT_EQ(stateAt(1.5f, 0.f, origin[2]), State::Free);

	// and back: the wrapped slices start unknown, the rest is unchanged
	grid.recenter(origin);
	EXPECT_EQ(stateAt(kWall, 0.f, origin[2]), State::Unknown);
	EXPECT_EQ(stateAt(1.5f, 0.f, origin[2]), State::Free);
}

TEST_F(ObstacleGridTest, ScrollingByAWholeWindowClearsEverything)
{
	insertWall(kWall, 2);
	const float far[3] {origin[0] + 20.f, origin[1], origin[2]};
	grid.recenter(far);
	grid.recenter(origin);
	EXPECT_EQ(stateAt(1.5f, 0.f, origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, OriginShiftMovesTheContent)
{
	insertWall(kWall, 2);
	// the local frame origin moved so every position reads 0.6 m further north
	const float delta[3] {0.6f, 0.f, 0.f};
	grid.shiftOrigin(delta);
	EXPECT_EQ(stateAt(kWall + 0.6f, 0.f, origin[2]), State::Occupied);
	EXPECT_EQ(stateAt(2.1f, 0.f, origin[2]), State::Free);
}

TEST_F(ObstacleGridTest, SectorDistancesFindTheWallAhead)
{
	insertWall(kWall, 2);

	float distance[kBins];
	sectorDistances(grid, origin, 0.f, band(), 4.65f, kBins, distance);

	// the nearest point of the wall voxels, at most one voxel short of the wall
	EXPECT_LE(distance[0], kWall - origin[0] + 1e-3f);
	EXPECT_GE(distance[0], kWall - origin[0] - kVoxel);
	// behind the vehicle was never observed
	EXPECT_TRUE(isnan(distance[36]));

	// facing east, the wall is on the left: sector -90 deg is 270 deg, index 54
	float rotated[kBins];
	sectorDistances(grid, origin, kPi / 2.f, band(), 4.65f, kBins, rotated);
	EXPECT_FLOAT_EQ(rotated[54], distance[0]);
	EXPECT_TRUE(isnan(rotated[0]));
}

TEST_F(ObstacleGridTest, SectorDistancesIgnoreTheFloorBelowTheBand)
{
	// rays down at 30 deg hitting the ground at down = 0, 2 m below
	for (int frame = 0; frame < 3; frame++) {
		for (int i = -5; i <= 5; i++) {
			const float az = i * 0.05f;
			const float el = 30.f * kPi / 180.f;
			const float direction[3] {cosf(el) *cosf(az), cosf(el) *sinf(az), sinf(el)};
			// end just under the surface, inside the voxel below down = 0
			grid.insertRay(origin, direction, (0.07f - origin[2]) / sinf(el), true);
		}
	}

	const float reach = (0.07f - origin[2]) / tanf(30.f * kPi / 180.f);
	ASSERT_EQ(stateAt(origin[0] + reach, origin[1], 0.07f), State::Occupied);

	float distance[kBins];
	sectorDistances(grid, origin, 0.f, band(), 4.65f, kBins, distance);
	// the rays crossed the band only within half a metre, so the ground is no obstacle but
	// nothing says the sector is clear further out
	EXPECT_TRUE(isnan(distance[0]));
}

TEST_F(ObstacleGridTest, SectorIsClearOnlyWhenObservedToMaxRange)
{
	// a level fan with no return, cleared to 6 m, past the 4.65 m the sectors reach
	for (int frame = 0; frame < 2; frame++) {
		for (int i = -40; i <= 40; i++) {
			const float az = i * 0.01f;
			const float direction[3] {cosf(az), sinf(az), 0.f};
			grid.insertRay(origin, direction, 6.f, false);
		}
	}

	float distance[kBins];
	sectorDistances(grid, origin, 0.f, band(), 4.65f, kBins, distance);
	EXPECT_TRUE(isinf(distance[0]));

	// the same fan cleared to 2 m only
	ObstacleGrid short_grid;
	ASSERT_TRUE(short_grid.allocate(64, 32));
	short_grid.setVoxelSize(kVoxel);
	short_grid.recenter(origin);

	for (int frame = 0; frame < 2; frame++) {
		for (int i = -40; i <= 40; i++) {
			const float az = i * 0.01f;
			const float direction[3] {cosf(az), sinf(az), 0.f};
			short_grid.insertRay(origin, direction, 2.f, false);
		}
	}

	sectorDistances(short_grid, origin, 0.f, bandAround(short_grid, origin, kRadius, kHalfHeight, 0.f), 4.65f, kBins, distance);
	EXPECT_TRUE(isnan(distance[0]));
}

TEST_F(ObstacleGridTest, SmallOriginShiftsAddUp)
{
	insertWall(kWall, 2);
	// three resets of under half a voxel each move the content 0.21 m, one voxel
	const float delta[3] {0.07f, 0.f, 0.f};

	for (int i = 0; i < 3; i++) {
		grid.shiftOrigin(delta);
	}

	EXPECT_EQ(stateAt(kWall + kVoxel, 0.f, origin[2]), State::Occupied);
}

TEST_F(ObstacleGridTest, ThresholdsKeepUnknownBetweenFreeAndOccupied)
{
	ObstacleGrid::Weights weights{};
	weights.free = 0;
	weights.occupied = 0;
	weights.clamp_min = -1;
	grid.setWeights(weights);
	EXPECT_EQ(grid.weights().free, -1);
	EXPECT_EQ(grid.weights().occupied, 1);
	EXPECT_EQ(stateAt(-3.f, 2.f, origin[2]), State::Unknown);
}

TEST_F(ObstacleGridTest, TileUsesTheZoneGeometry)
{
	// one row of three zones at -10, 0 and +10 deg azimuth
	range_image::Geometry geometry{};
	geometry.num_rows = 1;
	geometry.num_cols = 3;
	geometry.x_start = -10.f;
	geometry.x_step = 10.f;

	SensorPose pose{};
	memcpy(pose.origin, origin, sizeof(origin));
	// sensor looking east: sensor x is local east, y is local south, z is down
	const float rotation[9] {0.f, -1.f, 0.f,
				 1.f, 0.f, 0.f,
				 0.f, 0.f, 1.f
				};
	memcpy(pose.rotation, rotation, sizeof(rotation));

	const uint16_t ranges[3] {range_image::kRangeInvalid, 1000, range_image::kRangeNoReturn};
	Tile tile{0, ranges, 3, 0.002f, 0.05f, 4.f};

	EXPECT_EQ(insertTile(grid, geometry, pose, tile), 2);
	EXPECT_EQ(insertTile(grid, geometry, pose, tile), 2);
	// the centre zone saw something 2 m east
	EXPECT_EQ(stateAt(origin[0], origin[1] + 2.f, origin[2]), State::Occupied);
	// the +10 deg zone (to the right of east, so south of it) cleared to 4 m
	EXPECT_EQ(stateAt(origin[0] - 3.f * sinf(10.f * kPi / 180.f), origin[1] + 3.f * cosf(10.f * kPi / 180.f), origin[2]),
		  State::Free);
}

TEST_F(ObstacleGridTest, BandLeavesOutTheFloorUnderTheVehicleButNotWhatStandsOnIt)
{
	// 0.6 m over a floor, so a 0.5 m margin under the body reaches into it
	origin[2] = -0.62f;
	grid.recenter(origin);
	insertSlab(-1.5f, 2.5f, -1.5f, 1.5f, 0.07f);

	// a crate 2 m ahead standing 0.45 m tall
	for (float down = -0.38f; down < 0.f; down += kVoxel) {
		insertSlab(2.f, 2.3f, -0.3f, 0.3f, down);
	}

	const Band wide = band(0.5f);
	EXPECT_LT(wide.down_max, grid.voxelIndex(0.07f) - 1);
	EXPECT_GE(wide.down_max, grid.voxelIndex(origin[2]));

	float distance[kBins];
	sectorDistances(grid, origin, 0.f, wide, 4.65f, kBins, distance);
	EXPECT_NEAR(distance[0], 2.f - origin[0], kVoxel);
	// the floor beside it is no obstacle
	EXPECT_FALSE(isfinite(distance[18]));
}

TEST_F(ObstacleGridTest, SweepAheadStopsAtTheWall)
{
	insertWall(kWall, 2);
	const Sweep ahead = sweep(1.f, 0.f, 0.f);
	EXPECT_EQ(ahead.face, Sweep::Face::Side);
	// the body's front reaches the wall's voxels, resolved to one voxel and rounded down
	EXPECT_LE(ahead.contact, kWall - origin[0] - kRadius + 1e-3f);
	EXPECT_GE(ahead.contact, kWall - origin[0] - kRadius - 2.f * kVoxel);
	EXPECT_LE(ahead.observed, ahead.contact);
}

TEST_F(ObstacleGridTest, SweepsUpAndDownFindCeilingAndFloor)
{
	insertSlab(-1.f, 1.f, -1.f, 1.f, origin[2] - 1.2f);
	insertSlab(-1.f, 1.f, -1.f, 1.f, origin[2] + 1.5f);

	const Sweep up = sweep(0.f, 0.f, -1.f);
	EXPECT_EQ(up.face, Sweep::Face::Top);
	EXPECT_NEAR(up.contact, 1.2f - kHalfHeight - 0.5f * kVoxel, 1.5f * kVoxel);

	const Sweep down = sweep(0.f, 0.f, 1.f);
	EXPECT_EQ(down.face, Sweep::Face::Bottom);
	EXPECT_NEAR(down.contact, 1.5f - kHalfHeight - 0.5f * kVoxel, 1.5f * kVoxel);
}

TEST_F(ObstacleGridTest, SweepFindsACeilingAheadThatTheBandLeavesOut)
{
	// a ceiling from 1 m to 3 m ahead, 1 m up
	insertSlab(1.f, 3.f, -1.f, 1.f, origin[2] - 1.f);

	float distance[kBins];
	sectorDistances(grid, origin, 0.f, band(0.5f), 4.65f, kBins, distance);
	EXPECT_FALSE(isfinite(distance[0]) && distance[0] < 3.f);

	// climbing forward at 45 degrees the body's top reaches it
	const Sweep climb = sweep(1.f, 0.f, -1.f);
	EXPECT_EQ(climb.face, Sweep::Face::Top);
	EXPECT_NEAR(climb.contact, (1.f - kHalfHeight) * sqrtf(2.f), 2.f * kVoxel);

	// level, the body passes under it
	EXPECT_TRUE(isinf(sweep(1.f, 0.f, 0.f).contact));
}

TEST_F(ObstacleGridTest, SweepJustOverAFloorLeavesItOut)
{
	// just over a floor, the body's lower half in it until the band leaves the floor out
	origin[2] = -0.17f;
	grid.recenter(origin);
	insertSlab(-2.f, 3.f, -1.f, 1.f, 0.07f);

	EXPECT_TRUE(isinf(sweep(1.f, 0.f, 0.f).contact));
	const Sweep down = sweep(0.f, 0.f, 1.f);
	EXPECT_EQ(down.face, Sweep::Face::Bottom);
	EXPECT_LE(down.contact, kVoxel);
}

TEST_F(ObstacleGridTest, SweepObservedEndsAtUnknownVoxels)
{
	// nothing observed: the body is in unknown space after its first step
	EXPECT_LE(sweep(1.f, 0.f, 0.f).observed, kVoxel);

	// cleared to 2 m ahead by a fan from 1.5 m behind, as the vehicle sees its way while it flies
	const float behind[3] {origin[0] - 1.5f, origin[1], origin[2]};

	for (int frame = 0; frame < 2; frame++) {
		for (int i = -30; i <= 30; i++) {
			for (int j = -12; j <= 12; j++) {
				const float az = i * 0.01f;
				const float el = j * 0.02f;
				const float direction[3] {cosf(el) *cosf(az), cosf(el) *sinf(az), sinf(el)};
				grid.insertRay(behind, direction, 3.5f, false);
			}
		}
	}

	const Sweep ahead = sweep(1.f, 0.f, 0.f);
	EXPECT_TRUE(isinf(ahead.contact));
	EXPECT_GT(ahead.observed, 0.5f);
	EXPECT_LT(ahead.observed, 2.f);
}

TEST_F(ObstacleGridTest, SweepIgnoresWhatTheBodyStartsIn)
{
	// a voxel inside the body where it starts, as a return from a prop or noise would leave
	for (int frame = 0; frame < 2; frame++) {
		const float up[3] {0.f, 0.f, -1.f};
		const float below[3] {origin[0], origin[1], origin[2] + 0.5f};
		grid.insertRay(below, up, 0.5f, true);
	}

	ASSERT_EQ(stateAt(origin[0], origin[1], origin[2]), State::Occupied);
	EXPECT_TRUE(isinf(sweep(1.f, 0.f, 0.f).contact));
	EXPECT_TRUE(isinf(sweep(0.f, 0.f, -1.f).contact));
	EXPECT_TRUE(isinf(sweep(0.f, 0.f, 1.f).contact));
}

TEST_F(ObstacleGridTest, ClimbingDiagonallyIntoAWallMeetsItsSide)
{
	insertWall(1.075f, 2);

	for (float angle : {30.f, 40.f, 45.f, 50.f}) {
		const float rad = angle * kPi / 180.f;
		const Sweep climb = sweep(cosf(rad), 0.f, -sinf(rad));

		if (isfinite(climb.contact)) {
			EXPECT_EQ(climb.face, Sweep::Face::Side) << angle;
		}
	}
}

TEST_F(ObstacleGridTest, AWallBesideIsNeitherCeilingNorFloor)
{
	// a wall 0.5 m east of the vehicle, floor to well above it
	for (float down = origin[2] - 1.5f; down < origin[2] + 1.5f; down += kVoxel) {
		insertSlab(-1.f, 1.f, 0.6f, 0.6f, down);
	}

	ASSERT_EQ(stateAt(origin[0], 0.6f, origin[2]), State::Occupied);

	// swept up and down two voxels wider than the body, so the wall is under the sweep
	const Band body = band();
	const float up[3] {0.f, 0.f, -1.f};
	const float down[3] {0.f, 0.f, 1.f};
	EXPECT_TRUE(isinf(sweepBody(grid, origin, up, kRadius + 2.f * kVoxel, body, 2.f).contact));
	EXPECT_TRUE(isinf(sweepBody(grid, origin, down, kRadius + 2.f * kVoxel, body, 2.f).contact));

	// and the band keeps its height
	const Band wide = band(0.5f);
	EXPECT_EQ(wide.down_min, grid.voxelIndex(origin[2] - kHalfHeight - 0.5f));
	EXPECT_EQ(wide.down_max, grid.voxelIndex(origin[2] + kHalfHeight + 0.5f));
}
