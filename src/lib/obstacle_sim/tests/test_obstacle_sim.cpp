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

#include <lib/obstacle_sim/obstacle_sim.h>

#include <math.h>

using namespace obstacle_sim;

static WorldConfig treesConfig(int32_t seed)
{
	WorldConfig config{};
	config.type = WorldType::Trees;
	config.seed = seed;
	return config;
}

// a search over enough cells to hold several trunks at the default density
static constexpr float kSearchRadius = 30.f;
static constexpr int kMaxTrees = 256;

TEST(ObstacleSim, NoneHasNoTrunks)
{
	World world;
	world.configure(WorldConfig{});
	Tree trees[kMaxTrees];
	EXPECT_EQ(world.treesNear(0.f, 0.f, kSearchRadius, trees, kMaxTrees), 0);

	const float origin[3] {0.f, 0.f, -2.f};
	const float forward[3] {1.f, 0.f, 0.f};
	EXPECT_TRUE(isinf(world.raycast(origin, forward, 9.f)));
}

TEST(ObstacleSim, SameSeedSameTrunks)
{
	World a;
	World b;
	a.configure(treesConfig(42));
	b.configure(treesConfig(42));

	Tree trees_a[kMaxTrees];
	Tree trees_b[kMaxTrees];
	const int count = a.treesNear(0.f, 0.f, kSearchRadius, trees_a, kMaxTrees);
	ASSERT_GT(count, 10);
	ASSERT_EQ(b.treesNear(0.f, 0.f, kSearchRadius, trees_b, kMaxTrees), count);

	for (int i = 0; i < count; i++) {
		EXPECT_EQ(trees_a[i].north, trees_b[i].north);
		EXPECT_EQ(trees_a[i].east, trees_b[i].east);
		EXPECT_EQ(trees_a[i].radius, trees_b[i].radius);
	}

	World c;
	c.configure(treesConfig(43));
	Tree trees_c[kMaxTrees];
	const int count_c = c.treesNear(0.f, 0.f, kSearchRadius, trees_c, kMaxTrees);
	EXPECT_TRUE(count_c != count || fabsf(trees_c[0].north - trees_a[0].north) > 1e-6f);
}

TEST(ObstacleSim, TrunksStayInsideTheirCellAndClearOfHome)
{
	World world;
	const WorldConfig config = treesConfig(7);
	world.configure(config);

	for (int32_t cn = -10; cn < 10; cn++) {
		for (int32_t ce = -10; ce < 10; ce++) {
			Tree tree;

			if (!world.treeInCell(cn, ce, tree)) {
				continue;
			}

			EXPECT_GE(tree.north - tree.radius, cn * config.cell_size);
			EXPECT_LE(tree.north + tree.radius, (cn + 1) * config.cell_size);
			EXPECT_GE(tree.east - tree.radius, ce * config.cell_size);
			EXPECT_LE(tree.east + tree.radius, (ce + 1) * config.cell_size);
			EXPECT_GE(tree.radius, config.radius_min);
			EXPECT_LE(tree.radius, config.radius_max);
			EXPECT_GE(hypotf(tree.north, tree.east) - tree.radius, config.clear_radius);
		}
	}
}

TEST(ObstacleSim, RaycastHitsTheTrunkSurface)
{
	World world;
	world.configure(treesConfig(3));
	Tree trees[kMaxTrees];
	const int count = world.treesNear(0.f, 0.f, kSearchRadius, trees, kMaxTrees);
	ASSERT_GT(count, 0);

	for (int i = 0; i < count; i++) {
		const Tree &tree = trees[i];
		// from 2 m beside the trunk at 1 m height, looking at its centre
		const float origin[3] {tree.north - tree.radius - 2.f, tree.east, -1.f};
		const float direction[3] {1.f, 0.f, 0.f};
		const float range = world.raycast(origin, direction, 9.f);
		// a trunk in a neighbouring cell can sit between the two
		EXPECT_LE(range, 2.f + 1e-4f);
		EXPECT_NEAR(world.nearestTrunk(origin[0], origin[1], origin[2], kSearchRadius), fminf(range, 2.f), 2.f);
	}
}

TEST(ObstacleSim, RaycastOverTheTopMissesTheTrunk)
{
	World world;
	world.configure(treesConfig(3));
	Tree trees[kMaxTrees];
	ASSERT_GT(world.treesNear(0.f, 0.f, kSearchRadius, trees, kMaxTrees), 0);
	const Tree &tree = trees[0];

	// level ray above the top, then the same ray just below it
	const float above[3] {tree.north - tree.radius - 0.5f, tree.east, -tree.height - 0.1f};
	const float below[3] {tree.north - tree.radius - 0.5f, tree.east, -tree.height + 0.1f};
	const float direction[3] {1.f, 0.f, 0.f};
	EXPECT_GT(world.raycast(above, direction, 2.f * tree.radius + 0.4f), 2.f * tree.radius + 0.5f);
	EXPECT_NEAR(world.raycast(below, direction, 9.f), 0.5f, 1e-4f);
}

TEST(ObstacleSim, RaycastFromAboveATrunkHitsItsCapOnlyOverIt)
{
	World world;
	world.configure(treesConfig(3));
	Tree trees[kMaxTrees];
	ASSERT_GT(world.treesNear(0.f, 0.f, kSearchRadius, trees, kMaxTrees), 0);
	const Tree &tree = trees[0];

	// 1 m above the centre of the top: straight down hits the cap, 21 deg off vertical leaves the circle first
	const float origin[3] {tree.north, tree.east, -tree.height - 1.f};
	const float down[3] {0.f, 0.f, 1.f};
	EXPECT_NEAR(world.raycast(origin, down, 20.f), 1.f, 1e-4f);

	const float angle = 1.2f; // from the vertical, the ray moves 2.6 m sideways per metre down
	const float slanted[3] {sinf(angle), 0.f, cosf(angle)};
	EXPECT_GT(world.raycast(origin, slanted, 2.f), 1.f / cosf(angle) - 1e-3f);
}

TEST(ObstacleSim, RaycastHitsTheGround)
{
	World world;
	world.configure(WorldConfig{});
	const float origin[3] {0.f, 0.f, -2.f};
	const float s = sqrtf(0.5f);
	const float down_45[3] {s, 0.f, s};
	EXPECT_NEAR(world.raycast(origin, down_45, 9.f), 2.f * sqrtf(2.f), 1e-4f);
	EXPECT_TRUE(isinf(world.raycast(origin, down_45, 2.f)));

	const float under[3] {0.f, 0.f, 0.5f};
	EXPECT_EQ(world.raycast(under, down_45, 9.f), 0.f);
}

TEST(ObstacleSim, RaycastIsInsideItsCellsWhenDiagonal)
{
	// the DDA has to visit every cell the ray crosses, so a diagonal ray finds the same trunk
	// as a ray aimed straight at it
	World world;
	world.configure(treesConfig(11));
	Tree trees[kMaxTrees];
	const int count = world.treesNear(0.f, 0.f, kSearchRadius, trees, kMaxTrees);

	for (int i = 0; i < count; i++) {
		const Tree &tree = trees[i];
		const float origin[3] {tree.north - 3.f, tree.east - 3.f, -1.f};
		const float s = sqrtf(0.5f);
		const float direction[3] {s, s, 0.f};
		const float expected = sqrtf(18.f) - tree.radius;
		EXPECT_LE(world.raycast(origin, direction, 9.f), expected + 1e-3f);
	}
}

static World courseWorld()
{
	WorldConfig config{};
	config.type = WorldType::Course;
	World world;
	world.configure(config);
	return world;
}

TEST(ObstacleSim, OnlyTheCourseHasBoxes)
{
	World trees;
	trees.configure(treesConfig(0));
	EXPECT_EQ(trees.boxCount(), 0);
	const float point[3] {8.f, 0.f, -1.f};
	EXPECT_TRUE(isinf(trees.nearestBox(point)));

	const World course = courseWorld();
	EXPECT_GT(course.boxCount(), 0);
	Tree trunks[kMaxTrees];
	EXPECT_EQ(course.treesNear(0.f, 0.f, kSearchRadius, trunks, kMaxTrees), 0);
}

TEST(ObstacleSim, RaycastHitsTheNearFaceOfABox)
{
	const World course = courseWorld();
	const Box &fence = course.box(0);
	const float origin[3] {0.f, 0.f, -1.f};
	const float north[3] {1.f, 0.f, 0.f};
	EXPECT_NEAR(course.raycast(origin, north, 9.f), fence.min[0], 1e-4f);

	// over the top it reaches the next box or nothing
	const float high[3] {0.f, 0.f, fence.min[2] - 0.1f};
	EXPECT_GT(course.raycast(high, north, 9.f), fence.max[0]);

	// from above, down onto its top
	const float above[3] {0.5f * (fence.min[0] + fence.max[0]), 0.f, fence.min[2] - 2.f};
	const float down[3] {0.f, 0.f, 1.f};
	EXPECT_NEAR(course.raycast(above, down, 9.f), 2.f, 1e-4f);

	const float inside[3] {0.5f * (fence.min[0] + fence.max[0]), 0.f, -0.5f};
	EXPECT_EQ(course.raycast(inside, north, 9.f), 0.f);
}

TEST(ObstacleSim, NearestBoxIsTheDistanceToItsSurface)
{
	const World course = courseWorld();
	const Box &fence = course.box(0);
	int index = -1;

	const float in_front[3] {fence.min[0] - 1.5f, 0.f, -1.f};
	EXPECT_NEAR(course.nearestBox(in_front, &index), 1.5f, 1e-4f);
	EXPECT_EQ(index, 0);

	const float over_the_edge[3] {fence.max[0] + 0.3f, 0.f, fence.min[2] - 0.4f};
	EXPECT_NEAR(course.nearestBox(over_the_edge), 0.5f, 1e-4f);

	const float inside[3] {0.5f * (fence.min[0] + fence.max[0]), 0.f, -0.5f};
	EXPECT_EQ(course.nearestBox(inside), 0.f);
}
