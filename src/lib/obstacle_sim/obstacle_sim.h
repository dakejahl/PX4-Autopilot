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

/**
 * @file obstacle_sim.h
 *
 * Obstacle worlds for SIH on flat ground: procedural tree trunks as vertical cylinders, evaluated
 * from a seed with no storage, or a fixed course of boxes to fly over and under.
 *
 * Coordinates are local NED in meters from home, with the ground at down = 0. Space is cut
 * into square cells, each holding at most one trunk that lies wholly inside it, so a ray only
 * tests the cells it crosses and the first hit is the nearest.
 *
 * No PX4 headers and no heap, so SIH, the MAVSDK tests and the log replay tool build the same
 * world from the same parameters. The integer hash is bit exact across platforms.
 */

#pragma once

#include <stdint.h>

namespace obstacle_sim
{

enum class WorldType : int32_t {
	None = 0,
	Trees = 1,
	Course = 2,
};

struct WorldConfig {
	WorldType type{WorldType::None};
	int32_t seed{0};
	float cell_size{5.f};     ///< [m] side of the square cell that holds at most one trunk
	float density{0.5f};      ///< probability that a cell holds a trunk
	float clear_radius{4.f};  ///< [m] no trunk surface within this distance of home
	float radius_min{0.1f};   ///< [m] trunk radius
	float radius_max{0.3f};
	float height_min{5.f};    ///< [m] trunk height above the ground
	float height_max{12.f};
};

struct Tree {
	float north{0.f};
	float east{0.f};
	float radius{0.f};
	float height{0.f};
};

/** Axis-aligned box, local NED */
struct Box {
	float min[3];
	float max[3];
};

class World
{
public:
	/** Invalid sizes are clamped so every cell still fits its trunk. */
	void configure(const WorldConfig &config);

	const WorldConfig &config() const { return _config; }

	/** The trunk in cell (cell_north, cell_east), false if the cell is empty. */
	bool treeInCell(int32_t cell_north, int32_t cell_east, Tree &tree) const;

	/** Boxes of the Course world, none in the others */
	int boxCount() const;
	const Box &box(int index) const;

	/**
	 * Distance from a point to the nearest box, 0 inside one.
	 * @return distance, INFINITY without boxes
	 */
	float nearestBox(const float point[3], int *index = nullptr) const;

	/**
	 * Distance along a ray to the first trunk, box or the ground.
	 *
	 * @param origin    [m] local NED
	 * @param direction unit vector, local NED
	 * @param max_range [m]
	 * @return distance to the hit, INFINITY if nothing is hit within max_range, 0 if the origin
	 *         is inside a trunk, a box or under the ground
	 */
	float raycast(const float origin[3], const float direction[3], float max_range) const;

	/**
	 * Horizontal distance from a point to the surface of the nearest trunk that reaches its
	 * height, within search_radius. Negative inside a trunk.
	 *
	 * @return distance, INFINITY if no trunk is within search_radius
	 */
	float nearestTrunk(float north, float east, float down, float search_radius, Tree *nearest = nullptr) const;

	/** Trunks with their centre within radius of (north, east), up to max_trees. */
	int treesNear(float north, float east, float radius, Tree *trees, int max_trees) const;

private:
	int32_t cellIndex(float position) const;
	float trunkHit(const Tree &tree, const float origin[3], const float direction[3]) const;
	static float boxHit(const Box &box, const float origin[3], const float direction[3]);

	WorldConfig _config{};
};

} // namespace obstacle_sim
