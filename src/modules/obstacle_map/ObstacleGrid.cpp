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

#include "ObstacleGrid.hpp"

#include <math.h>
#include <string.h>

namespace obstacle_map
{

static constexpr float kDirectionEpsilon = 1e-6f;
static constexpr float kMinVoxelSize = 0.01f;
static constexpr int kMinSize = 4;
static constexpr float kPi = 3.14159265358979f;
// free columns closer than this many voxels do not mark a sector as observed
static constexpr float kMinKnownVoxels = 2.f;

static inline bool isPowerOfTwo(int value)
{
	return value > 0 && (value & (value - 1)) == 0;
}

static inline int clampi(int value, int low, int high)
{
	return (value < low) ? low : (value > high) ? high : value;
}

ObstacleGrid::~ObstacleGrid()
{
	delete[] _cells;
}

bool ObstacleGrid::allocate(int size_xy, int size_z)
{
	if (!isPowerOfTwo(size_xy) || !isPowerOfTwo(size_z) || size_xy < kMinSize || size_z < kMinSize) {
		return false;
	}

	delete[] _cells;
	_cells = new uint8_t[(size_t)size_xy * size_xy * size_z / 2];

	if (_cells == nullptr) {
		return false;
	}

	_size_xy = size_xy;
	_size_z = size_z;
	_mask_xy = (uint32_t)size_xy - 1;
	_mask_z = (uint32_t)size_z - 1;
	clear();
	return true;
}

void ObstacleGrid::setVoxelSize(float voxel_size)
{
	voxel_size = fmaxf(voxel_size, kMinVoxelSize);

	if (fabsf(voxel_size - _voxel_size) > kDirectionEpsilon) {
		_voxel_size = voxel_size;
		clear();
	}
}

void ObstacleGrid::setWeights(const Weights &weights)
{
	// unknown (0) has to sit strictly between free and occupied, and the clamp at or below free
	_weights = weights;
	_weights.occupied = clampi(_weights.occupied, 1, kClampMax);
	_weights.free = clampi(_weights.free, kClampMinLimit, -1);
	_weights.clamp_min = clampi(_weights.clamp_min, kClampMinLimit, _weights.free);
}

void ObstacleGrid::clear()
{
	if (_cells) {
		memset(_cells, 0, (size_t)_size_xy * _size_xy * _size_z / 2);
	}

	_centered = false;
}

int32_t ObstacleGrid::voxelIndex(float position) const
{
	return (int32_t)floorf(position / _voxel_size);
}

uint32_t ObstacleGrid::storageIndex(int32_t north, int32_t east, int32_t down) const
{
	// two's complement wrap, so negative indices land in the ring too
	const uint32_t n = (uint32_t)(north + _offset[0]) & _mask_xy;
	const uint32_t e = (uint32_t)(east + _offset[1]) & _mask_xy;
	const uint32_t d = (uint32_t)(down + _offset[2]) & _mask_z;
	return (n * (uint32_t)_size_xy + e) * (uint32_t)_size_z + d;
}

int ObstacleGrid::rawValue(uint32_t index) const
{
	const uint8_t byte = _cells[index >> 1];
	const int nibble = (index & 1u) ? (byte >> 4) : (byte & 0x0F);
	// sign extend the 4-bit value
	return (nibble ^ 0x08) - 0x08;
}

void ObstacleGrid::setRawValue(uint32_t index, int value)
{
	uint8_t &byte = _cells[index >> 1];
	const uint8_t nibble = (uint8_t)value & 0x0F;
	byte = (index & 1u) ? (uint8_t)((byte & 0x0F) | (nibble << 4)) : (uint8_t)((byte & 0xF0) | nibble);
}

bool ObstacleGrid::inWindow(int32_t north, int32_t east, int32_t down) const
{
	const int32_t half_xy = _size_xy / 2;
	const int32_t half_z = _size_z / 2;
	return _centered
	       && north >= _center[0] - half_xy && north < _center[0] + half_xy
	       && east >= _center[1] - half_xy && east < _center[1] + half_xy
	       && down >= _center[2] - half_z && down < _center[2] + half_z;
}

int ObstacleGrid::value(int32_t north, int32_t east, int32_t down) const
{
	if (!_cells || !inWindow(north, east, down)) {
		return 0;
	}

	return rawValue(storageIndex(north, east, down));
}

ObstacleGrid::State ObstacleGrid::classify(int value) const
{
	if (value >= _weights.occupied) {
		return State::Occupied;
	}

	if (value <= _weights.free) {
		return State::Free;
	}

	return State::Unknown;
}

void ObstacleGrid::addValue(const int32_t voxel[3], int delta)
{
	const uint32_t index = storageIndex(voxel[0], voxel[1], voxel[2]);
	setRawValue(index, clampi(rawValue(index) + delta, _weights.clamp_min, kClampMax));
}

void ObstacleGrid::clearSlice(int axis, int32_t index)
{
	const uint32_t size[3] {(uint32_t)_size_xy, (uint32_t)_size_xy, (uint32_t)_size_z};
	const uint32_t stride[3] {size[1] *size[2], size[2], 1};
	const uint32_t mask[3] {_mask_xy, _mask_xy, _mask_z};
	const uint32_t slice = (uint32_t)(index + _offset[axis]) & mask[axis];

	const int a = (axis + 1) % 3;
	const int b = (axis + 2) % 3;

	for (uint32_t i = 0; i < size[a]; i++) {
		for (uint32_t j = 0; j < size[b]; j++) {
			setRawValue(slice * stride[axis] + i * stride[a] + j * stride[b], 0);
		}
	}
}

void ObstacleGrid::recenter(const float position[3])
{
	if (!_cells) {
		return;
	}

	const int32_t target[3] {voxelIndex(position[0]), voxelIndex(position[1]), voxelIndex(position[2])};

	if (!_centered) {
		memcpy(_center, target, sizeof(_center));
		_centered = true;
		return;
	}

	for (int axis = 0; axis < 3; axis++) {
		const int32_t size = (axis < 2) ? _size_xy : _size_z;
		const int32_t delta = target[axis] - _center[axis];

		if (delta >= size || delta <= -size) {
			clear();
			memcpy(_center, target, sizeof(_center));
			_centered = true;
			return;
		}

		// the slices entering the window share storage with the ones leaving it
		if (delta > 0) {
			for (int32_t i = 0; i < delta; i++) {
				clearSlice(axis, _center[axis] + size / 2 + i);
			}

		} else {
			for (int32_t i = delta; i < 0; i++) {
				clearSlice(axis, _center[axis] - size / 2 + i);
			}
		}

		_center[axis] = target[axis];
	}
}

void ObstacleGrid::shiftOrigin(const float delta[3])
{
	for (int axis = 0; axis < 3; axis++) {
		// the part under a voxel carries over, so small resets add up instead of being lost
		const float shift = _shift_remainder[axis] + delta[axis];
		const int32_t voxels = (int32_t)lroundf(shift / _voxel_size);
		_shift_remainder[axis] = shift - voxels * _voxel_size;
		_offset[axis] -= voxels;
		_center[axis] += voxels;
	}
}

int ObstacleGrid::insertRay(const float origin[3], const float direction[3], float range, bool hit)
{
	if (!_cells || !(range > 0.f)) {
		return 0;
	}

	int32_t voxel[3] {voxelIndex(origin[0]), voxelIndex(origin[1]), voxelIndex(origin[2])};

	if (!inWindow(voxel[0], voxel[1], voxel[2])) {
		return 0;
	}

	// Amanatides and Woo voxel traversal, t is the distance along the ray
	int32_t step[3] {};
	float t_next[3] {};
	float t_delta[3] {};

	for (int axis = 0; axis < 3; axis++) {
		if (direction[axis] > kDirectionEpsilon) {
			step[axis] = 1;
			t_next[axis] = ((voxel[axis] + 1) * _voxel_size - origin[axis]) / direction[axis];
			t_delta[axis] = _voxel_size / direction[axis];

		} else if (direction[axis] < -kDirectionEpsilon) {
			step[axis] = -1;
			t_next[axis] = (voxel[axis] * _voxel_size - origin[axis]) / direction[axis];
			t_delta[axis] = -_voxel_size / direction[axis];

		} else {
			t_next[axis] = INFINITY;
			t_delta[axis] = INFINITY;
		}
	}

	// the miss on each voxel waits one step, so the voxel next to the endpoint can skip it
	int32_t pending[3] {};
	bool have_pending = false;
	int updated = 0;

	// a ray crosses at most one face per voxel along each axis of the window
	const int max_steps = 2 * _size_xy + _size_z + 1;

	for (int i = 0; i < max_steps; i++) {
		const int axis = (t_next[0] < t_next[1]) ? ((t_next[0] < t_next[2]) ? 0 : 2) : ((t_next[1] < t_next[2]) ? 1 : 2);

		if (t_next[axis] > range) {
			// the ray ends in this voxel
			if (hit) {
				addValue(voxel, _weights.hit);
				return updated + 1;
			}

			if (have_pending) {
				addValue(pending, -_weights.miss);
				updated++;
			}

			addValue(voxel, -_weights.miss);
			return updated + 1;
		}

		if (have_pending) {
			addValue(pending, -_weights.miss);
			updated++;
		}

		memcpy(pending, voxel, sizeof(pending));
		have_pending = true;

		voxel[axis] += step[axis];
		t_next[axis] += t_delta[axis];

		if (!inWindow(voxel[0], voxel[1], voxel[2])) {
			// leaving the window, the endpoint lies outside it
			const bool next_is_endpoint = hit
						      && voxel[0] == voxelIndex(origin[0] + direction[0] * range)
						      && voxel[1] == voxelIndex(origin[1] + direction[1] * range)
						      && voxel[2] == voxelIndex(origin[2] + direction[2] * range);

			if (!next_is_endpoint) {
				addValue(pending, -_weights.miss);
				updated++;
			}

			return updated;
		}
	}

	return updated;
}

int insertTile(ObstacleGrid &grid, const range_image::Geometry &geometry, const SensorPose &pose, const Tile &tile)
{
	const float *R = pose.rotation;
	int rays = 0;

	for (int k = 0; k < tile.num_ranges; k++) {
		const uint16_t count = tile.ranges[k];

		if (count == range_image::kRangeInvalid || count == range_image::kRangeBelowMin) {
			continue;
		}

		float sensor[3];

		if (!range_image::zoneDirection(geometry, tile.first_zone + k, sensor)) {
			continue;
		}

		const float local[3] {
			R[0] *sensor[0] + R[1] *sensor[1] + R[2] *sensor[2],
			R[3] *sensor[0] + R[4] *sensor[1] + R[5] *sensor[2],
			R[6] *sensor[0] + R[7] *sensor[1] + R[8] *sensor[2],
		};

		if (count == range_image::kRangeNoReturn) {
			grid.insertRay(pose.origin, local, range_image::radialRange(geometry, sensor, tile.range_max), false);

		} else {
			const float range = range_image::radialRange(geometry, sensor, count * tile.lsb);

			if (!(range >= tile.range_min)) {
				continue;
			}

			grid.insertRay(pose.origin, local, range, true);
		}

		rays++;
	}

	return rays;
}

// Whether any voxel of a layer is occupied within radius of (north, east) [m], in a column that is
// free at layer own. A column occupied where the vehicle is is a wall beside it, not a floor or
// ceiling.
static bool layerOccupied(const ObstacleGrid &grid, float north, float east, float radius, int32_t down, int32_t own)
{
	const float voxel = grid.voxelSize();

	for (int32_t n = grid.voxelIndex(north - radius); n <= grid.voxelIndex(north + radius); n++) {
		const float dn = fminf(fmaxf(north, n * voxel), (n + 1) * voxel) - north;

		for (int32_t e = grid.voxelIndex(east - radius); e <= grid.voxelIndex(east + radius); e++) {
			const float de = fminf(fmaxf(east, e * voxel), (e + 1) * voxel) - east;

			if (dn * dn + de * de <= radius * radius && grid.state(n, e, down) == ObstacleGrid::State::Occupied
			    && grid.state(n, e, own) != ObstacleGrid::State::Occupied) {
				return true;
			}
		}
	}

	return false;
}

Band bandAround(const ObstacleGrid &grid, const float position[3], float footprint_radius, float half_height,
		float margin)
{
	const int32_t own = grid.voxelIndex(position[2]);
	Band band{grid.voxelIndex(position[2] - half_height - margin), grid.voxelIndex(position[2] + half_height + margin)};

	if (!grid.allocated()) {
		return band;
	}

	// a voxel past the footprint, so a surface seen only around its edge still counts
	const float radius = footprint_radius + grid.voxelSize();

	for (int32_t d = own + 1; d <= band.down_max; d++) {
		if (layerOccupied(grid, position[0], position[1], radius, d, own)) {
			band.down_max = (d - 2 > own) ? d - 2 : own;
			break;
		}
	}

	for (int32_t d = own - 1; d >= band.down_min; d--) {
		if (layerOccupied(grid, position[0], position[1], radius, d, own)) {
			band.down_min = (d + 2 < own) ? d + 2 : own;
			break;
		}
	}

	return band;
}

void sectorDistances(const ObstacleGrid &grid, const float position[3], float yaw, const Band &band,
		     float max_range, int num_bins, float *distance)
{
	for (int b = 0; b < num_bins; b++) {
		distance[b] = NAN;
	}

	if (!grid.allocated() || num_bins <= 0 || num_bins > kMaxSectors) {
		return;
	}

	// whether a sector has free columns, and whether it has unknown ones, beyond the vehicle's own
	static constexpr uint8_t SEEN_FREE = 1;
	static constexpr uint8_t SEEN_UNKNOWN = 2;
	uint8_t seen[kMaxSectors] {};

	const float voxel = grid.voxelSize();
	const float half_diagonal = voxel * 0.70710678f;
	const float bin_width = 2.f * kPi / num_bins;
	const int32_t *center = grid.center();
	const int32_t half_xy = grid.sizeXY() / 2;
	const int32_t half_z = grid.sizeZ() / 2;

	int32_t down_min = band.down_min;
	int32_t down_max = band.down_max;
	down_min = (down_min < center[2] - half_z) ? center[2] - half_z : down_min;
	down_max = (down_max > center[2] + half_z - 1) ? center[2] + half_z - 1 : down_max;

	for (int32_t n = center[0] - half_xy; n < center[0] + half_xy; n++) {
		const float north_low = n * voxel;
		const float dn_near = fminf(fmaxf(position[0], north_low), north_low + voxel) - position[0];
		const float dn_center = north_low + 0.5f * voxel - position[0];

		for (int32_t e = center[1] - half_xy; e < center[1] + half_xy; e++) {
			const float east_low = e * voxel;
			const float de_near = fminf(fmaxf(position[1], east_low), east_low + voxel) - position[1];
			const float near = sqrtf(dn_near * dn_near + de_near * de_near);

			if (near > max_range) {
				continue;
			}

			bool occupied = false;
			bool known = false;

			for (int32_t d = down_min; d <= down_max; d++) {
				const ObstacleGrid::State state = grid.state(n, e, d);

				if (state == ObstacleGrid::State::Occupied) {
					occupied = true;
					break;
				}

				known |= (state == ObstacleGrid::State::Free);
			}

			const float de_center = east_low + 0.5f * voxel - position[1];
			const float center_distance = sqrtf(dn_center * dn_center + de_center * de_center);

			if (!occupied && center_distance < kMinKnownVoxels * voxel) {
				// The columns around the vehicle span many sectors each, and the rays of a forward
				// sensor cross them all, so only columns further out tell a sector apart.
				continue;
			}

			const float bearing = atan2f(de_center, dn_center) - yaw;

			if (!occupied) {
				const int bin = (((int)floorf(bearing / bin_width + 0.5f) % num_bins) + num_bins) % num_bins;
				seen[bin] |= known ? SEEN_FREE : SEEN_UNKNOWN;
				continue;
			}

			// an obstacle counts in every sector its footprint overlaps, seen from the vehicle
			int first = 0;
			int last = num_bins - 1;

			if (center_distance > half_diagonal) {
				const float half_width = asinf(half_diagonal / center_distance);
				first = (int)floorf((bearing - half_width) / bin_width + 0.5f);
				last = (int)floorf((bearing + half_width) / bin_width + 0.5f);
			}

			for (int b = first; b <= last; b++) {
				const int bin = ((b % num_bins) + num_bins) % num_bins;

				// false for NAN too
				if (!(distance[bin] <= near)) {
					distance[bin] = near;
				}
			}
		}
	}

	// clear only when observed out to max_range, so the sector does not promise space nobody has seen
	for (int b = 0; b < num_bins; b++) {
		if (__builtin_isnan(distance[b]) && seen[b] == SEEN_FREE) {
			distance[b] = INFINITY;
		}
	}
}

Sweep sweepBody(const ObstacleGrid &grid, const float position[3], const float direction[3], float radius,
		const Band &body, float max_distance)
{
	Sweep sweep{INFINITY, fmaxf(max_distance, 0.f), Sweep::Face::Side};

	if (!grid.allocated() || !(max_distance > 0.f)) {
		return sweep;
	}

	const float voxel = grid.voxelSize();
	// the body's extent along down from its position, as the layers it starts in
	const float top = body.down_min * voxel - position[2];
	const float bottom = (body.down_max + 1) * voxel - position[2];
	// so an edge on a voxel boundary does not overlap the voxel beyond it
	const float inset = 1e-3f * voxel;

	struct Shape {
		float north;
		float east;
		int32_t n0, n1, e0, e1, d0, d1;
	};

	const auto shape_at = [&](float travel) {
		Shape shape{};
		shape.north = position[0] + direction[0] * travel;
		shape.east = position[1] + direction[1] * travel;
		const float down = position[2] + direction[2] * travel;
		shape.n0 = grid.voxelIndex(shape.north - radius);
		shape.n1 = grid.voxelIndex(shape.north + radius);
		shape.e0 = grid.voxelIndex(shape.east - radius);
		shape.e1 = grid.voxelIndex(shape.east + radius);
		shape.d0 = grid.voxelIndex(down + top + inset);
		shape.d1 = grid.voxelIndex(down + bottom - inset);
		return shape;
	};

	// whether column (n, e) is under the body's circle
	const auto covers = [&](const Shape & shape, int32_t n, int32_t e) {
		if (n < shape.n0 || n > shape.n1 || e < shape.e0 || e > shape.e1) {
			return false;
		}

		const float dn = fminf(fmaxf(shape.north, n * voxel), (n + 1) * voxel) - shape.north;
		const float de = fminf(fmaxf(shape.east, e * voxel), (e + 1) * voxel) - shape.east;
		return dn * dn + de * de <= radius * radius;
	};

	Shape previous = shape_at(0.f);
	bool unknown_seen = false;

	// Columns the body starts beside an obstacle in, as a wall next to it, are left out altogether:
	// moving up or down along them is not moving towards them.
	static constexpr int kMaxStartColumns = 32;
	const int start_columns_n = previous.n1 - previous.n0 + 1;
	const int start_columns_e = previous.e1 - previous.e0 + 1;
	uint32_t started_beside[kMaxStartColumns] {}; // a bit per column, east within north
	const bool track_start = start_columns_n <= kMaxStartColumns && start_columns_e <= kMaxStartColumns;

	if (track_start) {
		for (int32_t n = previous.n0; n <= previous.n1; n++) {
			for (int32_t e = previous.e0; e <= previous.e1; e++) {
				if (!covers(previous, n, e)) {
					continue;
				}

				for (int32_t d = previous.d0; d <= previous.d1; d++) {
					if (grid.state(n, e, d) == ObstacleGrid::State::Occupied) {
						started_beside[n - previous.n0] |= 1u << (e - previous.e0);
						break;
					}
				}
			}
		}
	}

	const Shape start = previous;
	const auto beside_at_start = [&](int32_t n, int32_t e) {
		return track_start && n >= start.n0 && n <= start.n1 && e >= start.e0 && e <= start.e1
		       && (started_beside[n - start.n0] & (1u << (e - start.e0)));
	};
	const int steps = (int)ceilf(max_distance / voxel);

	for (int i = 1; i <= steps; i++) {
		const float travelled = (i - 1) * voxel;
		const Shape current = shape_at(fminf(i * voxel, max_distance));

		for (int32_t n = current.n0; n <= current.n1; n++) {
			for (int32_t e = current.e0; e <= current.e1; e++) {
				if (!covers(current, n, e) || beside_at_start(n, e)) {
					continue;
				}

				// only voxels this step moves the body into, so the start and what was checked are skipped
				const bool covered = covers(previous, n, e);

				for (int32_t d = current.d0; d <= current.d1; d++) {
					if (covered && d >= previous.d0 && d <= previous.d1) {
						continue;
					}

					const ObstacleGrid::State state = grid.state(n, e, d);

					// a layer the body already spanned was reached sideways
					const Sweep::Face face = (d >= previous.d0 && d <= previous.d1) ? Sweep::Face::Side
								 : (d < previous.d0) ? Sweep::Face::Top : Sweep::Face::Bottom;

					if (state == ObstacleGrid::State::Occupied && !(sweep.contact <= travelled)) {
						sweep.contact = travelled;
						sweep.face = face;

					} else if (state == ObstacleGrid::State::Occupied && face == Sweep::Face::Side) {
						// a wall met in the same step as its top or bottom edge is a wall
						sweep.face = face;

					} else if (state == ObstacleGrid::State::Unknown && !unknown_seen) {
						sweep.observed = travelled;
						unknown_seen = true;
					}
				}
			}
		}

		if (sweep.contact < INFINITY) {
			sweep.observed = fminf(sweep.observed, sweep.contact);
			return sweep;
		}

		previous = current;
	}

	return sweep;
}

void countStates(const ObstacleGrid &grid, uint32_t &occupied, uint32_t &free)
{
	occupied = 0;
	free = 0;

	if (!grid.allocated()) {
		return;
	}

	const int32_t *center = grid.center();
	const int32_t half_xy = grid.sizeXY() / 2;
	const int32_t half_z = grid.sizeZ() / 2;

	for (int32_t n = center[0] - half_xy; n < center[0] + half_xy; n++) {
		for (int32_t e = center[1] - half_xy; e < center[1] + half_xy; e++) {
			for (int32_t d = center[2] - half_z; d < center[2] + half_z; d++) {
				const ObstacleGrid::State state = grid.state(n, e, d);
				occupied += (state == ObstacleGrid::State::Occupied);
				free += (state == ObstacleGrid::State::Free);
			}
		}
	}
}

} // namespace obstacle_map
