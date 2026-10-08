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
 * @file replay_core.cpp
 *
 * C entry points into the obstacle map and the SIH obstacle world for replay.py, which builds
 * this file together with their sources, so a replay runs the same code as the vehicle.
 */

#include <lib/obstacle_sim/obstacle_sim.h>
#include <lib/range_image/RangeImageGeometry.hpp>
#include <modules/obstacle_map/ObstacleGrid.hpp>

#include <string.h>

using obstacle_map::ObstacleGrid;

extern "C" {

	void *grid_create(int size_xy, int size_z, float voxel_size, const int weights[5])
	{
		ObstacleGrid *grid = new ObstacleGrid();

		if (!grid->allocate(size_xy, size_z)) {
			delete grid;
			return nullptr;
		}

		grid->setVoxelSize(voxel_size);
		ObstacleGrid::Weights w{};
		w.hit = weights[0];
		w.miss = weights[1];
		w.occupied = weights[2];
		w.free = weights[3];
		w.clamp_min = weights[4];
		grid->setWeights(w);
		return grid;
	}

	void grid_destroy(void *grid)
	{
		delete static_cast<ObstacleGrid *>(grid);
	}

	void grid_recenter(void *grid, const float position[3])
	{
		static_cast<ObstacleGrid *>(grid)->recenter(position);
	}

	void grid_shift_origin(void *grid, const float delta[3])
	{
		static_cast<ObstacleGrid *>(grid)->shiftOrigin(delta);
	}

	void grid_clear(void *grid)
	{
		static_cast<ObstacleGrid *>(grid)->clear();
	}

	/**
	 * Geometry as in RangeImageInfo: int fields num_rows, num_cols, projection, range_type, zone_order,
	 * float fields x_start, x_step, y_start, y_step. Pose: origin[3] then a row-major rotation[9],
	 * sensor to local NED.
	 */
	int grid_insert_tile(void *grid, const int geometry_int[5], const float geometry_float[4], const float *row_angle,
			     int row_angle_count, const float pose[12], unsigned first_zone, const unsigned short *ranges, int num_ranges,
			     float lsb, float range_min, float range_max)
	{
		range_image::Geometry geometry{};
		geometry.num_rows = geometry_int[0];
		geometry.num_cols = geometry_int[1];
		geometry.projection = geometry_int[2];
		geometry.range_type = geometry_int[3];
		geometry.zone_order = geometry_int[4];
		geometry.x_start = geometry_float[0];
		geometry.x_step = geometry_float[1];
		geometry.y_start = geometry_float[2];
		geometry.y_step = geometry_float[3];
		geometry.row_angle = row_angle;
		geometry.row_angle_count = row_angle_count;

		obstacle_map::SensorPose sensor_pose{};
		memcpy(sensor_pose.origin, pose, sizeof(sensor_pose.origin));
		memcpy(sensor_pose.rotation, pose + 3, sizeof(sensor_pose.rotation));

		const obstacle_map::Tile tile{first_zone, ranges, num_ranges, lsb, range_min, range_max};
		return obstacle_map::insertTile(*static_cast<ObstacleGrid *>(grid), geometry, sensor_pose, tile);
	}

	/** Every occupied voxel in the window, as local voxel indices (north, east, down), up to max_voxels */
	int grid_occupied(void *grid_handle, int *voxels, int max_voxels)
	{
		const ObstacleGrid &grid = *static_cast<ObstacleGrid *>(grid_handle);
		const int32_t *center = grid.center();
		const int32_t half_xy = grid.sizeXY() / 2;
		const int32_t half_z = grid.sizeZ() / 2;
		int count = 0;

		for (int32_t n = center[0] - half_xy; n < center[0] + half_xy; n++) {
			for (int32_t e = center[1] - half_xy; e < center[1] + half_xy; e++) {
				for (int32_t d = center[2] - half_z; d < center[2] + half_z; d++) {
					if (grid.state(n, e, d) == ObstacleGrid::State::Occupied && count < max_voxels) {
						voxels[3 * count] = n;
						voxels[3 * count + 1] = e;
						voxels[3 * count + 2] = d;
						count++;
					}
				}
			}
		}

		return count;
	}

	/** 0 unknown, 1 free, 2 occupied */
	int grid_state(void *grid, int north, int east, int down)
	{
		return (int)static_cast<ObstacleGrid *>(grid)->state(north, east, down);
	}

	void grid_center(void *grid, int center[3])
	{
		const int32_t *c = static_cast<ObstacleGrid *>(grid)->center();
		center[0] = c[0];
		center[1] = c[1];
		center[2] = c[2];
	}

	void grid_sector_distances(void *grid, const float position[3], float yaw, float footprint_radius, float half_height,
				   float margin, float max_range, int num_bins, float *distance)
	{
		const ObstacleGrid &g = *static_cast<ObstacleGrid *>(grid);
		const obstacle_map::Band band = obstacle_map::bandAround(g, position, footprint_radius, half_height, margin);
		obstacle_map::sectorDistances(g, position, yaw, band, max_range, num_bins, distance);
	}

	void *world_create(int type, int seed, float cell_size, float density, float clear_radius)
	{
		obstacle_sim::World *world = new obstacle_sim::World();
		obstacle_sim::WorldConfig config{};
		config.type = static_cast<obstacle_sim::WorldType>(type);
		config.seed = seed;
		config.cell_size = cell_size;
		config.density = density;
		config.clear_radius = clear_radius;
		world->configure(config);
		return world;
	}

	void world_destroy(void *world)
	{
		delete static_cast<obstacle_sim::World *>(world);
	}

	/** Trunks near a point, as (north, east, radius, height) quadruples */
	int world_trees_near(void *world, float north, float east, float radius, float *trees, int max_trees)
	{
		obstacle_sim::Tree found[256];
		const int count = static_cast<obstacle_sim::World *>(world)->treesNear(north, east, radius, found,
				  max_trees < 256 ? max_trees : 256);

		for (int i = 0; i < count; i++) {
			trees[4 * i] = found[i].north;
			trees[4 * i + 1] = found[i].east;
			trees[4 * i + 2] = found[i].radius;
			trees[4 * i + 3] = found[i].height;
		}

		return count;
	}

	float world_raycast(void *world, const float origin[3], const float direction[3], float max_range)
	{
		return static_cast<obstacle_sim::World *>(world)->raycast(origin, direction, max_range);
	}

	float world_nearest_trunk(void *world, float north, float east, float down, float search_radius)
	{
		return static_cast<obstacle_sim::World *>(world)->nearestTrunk(north, east, down, search_radius);
	}

	int world_box_count(void *world)
	{
		return static_cast<obstacle_sim::World *>(world)->boxCount();
	}

	/** Corners of a box as (min north, east, down, max north, east, down) */
	void world_box(void *world, int index, float corners[6])
	{
		const obstacle_sim::Box &box = static_cast<obstacle_sim::World *>(world)->box(index);

		for (int axis = 0; axis < 3; axis++) {
			corners[axis] = box.min[axis];
			corners[3 + axis] = box.max[axis];
		}
	}

	float world_nearest_box(void *world, const float point[3])
	{
		return static_cast<obstacle_sim::World *>(world)->nearestBox(point);
	}

} // extern "C"
