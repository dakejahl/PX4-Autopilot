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
 * @file ObstacleGrid.hpp
 *
 * Rolling occupancy grid in the local NED frame with a 4-bit log-odds value per voxel.
 *
 * The window is a fixed number of voxels per axis centred on the vehicle and indexed as a 3D
 * ring buffer. When the vehicle crosses a voxel boundary the window scrolls by one voxel, and
 * the slice that wraps around starts again as unknown.
 *
 * No PX4 headers, so the log replay tool builds the same code on the host.
 */

#pragma once

#include <stdint.h>

#include <lib/range_image/RangeImageGeometry.hpp>

namespace obstacle_map
{

class ObstacleGrid
{
public:
	struct Weights {
		int hit{2};        ///< added to the voxel a return ends in
		int miss{1};       ///< subtracted from each voxel a ray passes through
		int occupied{3};   ///< a voxel at or above this is occupied
		int free{-2};      ///< a voxel at or below this is free
		int clamp_min{-4}; ///< lower clamp, misses never take a voxel below it
	};

	static constexpr int kClampMax = 7;   // largest signed 4-bit value
	static constexpr int kClampMinLimit = -8;

	enum class State : uint8_t {
		Unknown,
		Free,
		Occupied,
	};

	ObstacleGrid() = default;
	~ObstacleGrid();

	ObstacleGrid(const ObstacleGrid &) = delete;
	ObstacleGrid &operator=(const ObstacleGrid &) = delete;

	/**
	 * Allocate the window, size_xy voxels along north and east and size_z along down, each a
	 * power of two.
	 * @return false for an invalid size or a failed allocation
	 */
	bool allocate(int size_xy, int size_z);

	bool allocated() const { return _cells != nullptr; }
	int sizeXY() const { return _size_xy; }
	int sizeZ() const { return _size_z; }
	float voxelSize() const { return _voxel_size; }
	const Weights &weights() const { return _weights; }

	/** A new voxel size forgets the map. */
	void setVoxelSize(float voxel_size);
	void setWeights(const Weights &weights);

	/** Every voxel back to unknown */
	void clear();

	/** Scroll the window so the voxel holding position [m] is its centre voxel. */
	void recenter(const float position[3]);

	/** Move the content with the local frame after its origin moved by delta [m], in whole voxels. */
	void shiftOrigin(const float delta[3]);

	/**
	 * Walk a ray from origin along direction (unit, local NED) through the window.
	 *
	 * With hit the voxel the ray ends in takes a hit and every voxel before it a miss, except
	 * the one next to it, so rays grazing a surface do not erode it. Without hit every voxel up
	 * to range takes a miss.
	 *
	 * @return number of voxels updated
	 */
	int insertRay(const float origin[3], const float direction[3], float range, bool hit);

	int32_t voxelIndex(float position) const;
	const int32_t *center() const { return _center; }
	bool inWindow(int32_t north, int32_t east, int32_t down) const;

	/** Log-odds value of the voxel at local voxel indices, 0 outside the window */
	int value(int32_t north, int32_t east, int32_t down) const;
	State state(int32_t north, int32_t east, int32_t down) const { return classify(value(north, east, down)); }
	State classify(int value) const;

private:
	uint32_t storageIndex(int32_t north, int32_t east, int32_t down) const;
	int rawValue(uint32_t index) const;
	void setRawValue(uint32_t index, int value);
	void addValue(const int32_t voxel[3], int delta);
	void clearSlice(int axis, int32_t index);

	uint8_t *_cells{nullptr};
	int _size_xy{0};
	int _size_z{0};
	uint32_t _mask_xy{0};
	uint32_t _mask_z{0};
	float _voxel_size{0.15f};
	Weights _weights{};

	bool _centered{false};
	int32_t _center[3] {}; // local voxel index of the window centre
	int32_t _offset[3] {}; // added to a local voxel index before wrapping into storage
	float _shift_remainder[3] {}; // [m] origin shifts not yet applied, less than half a voxel
};

/** Pose of a sensor in the local NED frame */
struct SensorPose {
	float origin[3];   ///< [m]
	float rotation[9]; ///< sensor frame to local NED, row-major
};

/** One RangeImage tile with its range encoding from RangeImageInfo */
struct Tile {
	uint32_t first_zone;
	const uint16_t *ranges;
	int num_ranges;
	float lsb;       ///< [m] value of one count
	float range_min; ///< [m]
	float range_max; ///< [m]
};

/**
 * Insert every zone of a tile. A return is a hit at its range, RANGE_NO_RETURN clears to
 * range_max, and RANGE_BELOW_MIN and RANGE_INVALID carry no position, so they are skipped.
 * @return number of rays inserted
 */
int insertTile(ObstacleGrid &grid, const range_image::Geometry &geometry, const SensorPose &pose, const Tile &tile);

static constexpr int kMaxSectors = 72;

/** Voxel layers along down, both inclusive */
struct Band {
	int32_t down_min;
	int32_t down_max;
};

/**
 * Layers of a vehicle of half_height at position, widened by margin above and below, but
 * stopping two layers short of an occupied voxel directly over or under the footprint, and never
 * short of the vehicle's own layer. The ground the vehicle flies over and a ceiling it flies
 * under are then outside the band, with a layer to spare for noise on their surface.
 */
Band bandAround(const ObstacleGrid &grid, const float position[3], float footprint_radius, float half_height,
		float margin);

/**
 * Nearest occupied voxel column in each of num_bins (at most kMaxSectors) horizontal sectors
 * around the vehicle.
 *
 * Only voxels in the band and within max_range horizontally count. Sector b is centred on
 * yaw + b * 360 / num_bins degrees, clockwise seen from above.
 *
 * @param distance [m] per sector: horizontal distance to the nearest point of the nearest
 *        occupied column, INFINITY if every column in it out to max_range is free, NAN otherwise
 */
void sectorDistances(const ObstacleGrid &grid, const float position[3], float yaw, const Band &band,
		     float max_range, int num_bins, float *distance);

struct Sweep {
	enum class Face : uint8_t {
		Side,
		Top,
		Bottom,
	};

	float contact;  ///< [m] travel before the body overlaps an occupied voxel, INFINITY if it does not within max_distance
	float observed; ///< [m] travel through voxels all observed free, at most max_distance
	Face face;      ///< side of the body the contact is on
};

/**
 * Move the vehicle's body, a vertical cylinder of radius spanning the layers of body, from
 * position along direction (unit, local NED), and find where it first overlaps an occupied
 * voxel and where an unknown one. Voxels the body overlaps where it starts are ignored. The
 * travel is resolved to one voxel, rounded down.
 */
Sweep sweepBody(const ObstacleGrid &grid, const float position[3], const float direction[3], float radius,
		const Band &body, float max_distance);

/** Occupied and free voxels in the window */
void countStates(const ObstacleGrid &grid, uint32_t &occupied, uint32_t &free);

} // namespace obstacle_map
