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

#include "obstacle_sim.h"

#include <math.h>

namespace obstacle_sim
{

// gap between a trunk and its cell edge, so neighbouring trunks are at least twice this apart
static constexpr float kCellMargin = 0.25f;
static constexpr float kMinRadius = 0.01f;
static constexpr float kDirectionEpsilon = 1e-6f;

// Northwards from home: a fence to climb over, then a roof to fly and climb under.
static constexpr Box kCourse[] {
	{{8.f, -10.f, -1.5f}, {8.2f, 10.f, 0.f}},
	{{14.f, -10.f, -4.3f}, {30.f, 10.f, -4.f}},
};
static constexpr int kCourseBoxes = sizeof(kCourse) / sizeof(kCourse[0]);

// murmur3 style hash of a cell, uint32 multiplies and xor shifts only, so it is bit exact everywhere
static inline uint32_t hashCell(int32_t ix, int32_t iy, int32_t seed)
{
	uint32_t h = (uint32_t)ix * 0x27d4eb2du;
	h ^= (uint32_t)iy * 0x9e3779b1u;
	h ^= (uint32_t)seed * 0x85ebca6bu;
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return h;
}

// the k-th independent draw in [0, 1) from a cell hash, exact in float from the upper 24 bits
static inline float draw(uint32_t cell_hash, uint32_t k)
{
	uint32_t h = cell_hash + k * 0x9e3779b9u;
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return (float)(h >> 8) * (1.f / 16777216.f);
}

static inline float clampf(float value, float low, float high)
{
	return (value < low) ? low : (value > high) ? high : value;
}

void World::configure(const WorldConfig &config)
{
	_config = config;
	_config.radius_min = fmaxf(_config.radius_min, kMinRadius);
	_config.radius_max = fmaxf(_config.radius_max, _config.radius_min);
	_config.cell_size = fmaxf(_config.cell_size, 2.f * (_config.radius_max + kCellMargin));
	_config.density = clampf(_config.density, 0.f, 1.f);
	_config.clear_radius = fmaxf(_config.clear_radius, 0.f);
	_config.height_min = fmaxf(_config.height_min, 0.f);
	_config.height_max = fmaxf(_config.height_max, _config.height_min);
}

int32_t World::cellIndex(float position) const
{
	return (int32_t)floorf(position / _config.cell_size);
}

bool World::treeInCell(int32_t cell_north, int32_t cell_east, Tree &tree) const
{
	if (_config.type != WorldType::Trees) {
		return false;
	}

	const uint32_t h = hashCell(cell_north, cell_east, _config.seed);

	if (draw(h, 0) >= _config.density) {
		return false;
	}

	const float radius = _config.radius_min + draw(h, 1) * (_config.radius_max - _config.radius_min);
	const float margin = radius + kCellMargin;
	const float span = _config.cell_size - 2.f * margin;

	tree.radius = radius;
	tree.north = (float)cell_north * _config.cell_size + margin + draw(h, 2) * span;
	tree.east = (float)cell_east * _config.cell_size + margin + draw(h, 3) * span;
	tree.height = _config.height_min + draw(h, 4) * (_config.height_max - _config.height_min);

	const float keep_out = _config.clear_radius + radius;
	return tree.north * tree.north + tree.east * tree.east >= keep_out * keep_out;
}

float World::trunkHit(const Tree &tree, const float origin[3], const float direction[3]) const
{
	const float dn = origin[0] - tree.north;
	const float de = origin[1] - tree.east;
	const float top = -tree.height;
	const float c = dn * dn + de * de - tree.radius * tree.radius;

	const float a = direction[0] * direction[0] + direction[1] * direction[1];

	if (c <= 0.f) {
		// inside the trunk, or above it
		if (origin[2] >= top) {
			return 0.f;
		}

		if (direction[2] <= kDirectionEpsilon) {
			return INFINITY;
		}

		// the cap is hit only if the ray comes down to it before leaving the circle
		const float t_cap = (top - origin[2]) / direction[2];

		if (a < kDirectionEpsilon) {
			return t_cap;
		}

		const float b = dn * direction[0] + de * direction[1];
		const float t_exit = (-b + sqrtf(b * b - a * c)) / a;
		return (t_cap <= t_exit) ? t_cap : INFINITY;
	}

	if (a < kDirectionEpsilon) {
		return INFINITY;
	}

	const float b = dn * direction[0] + de * direction[1];
	const float discriminant = b * b - a * c;

	if (discriminant < 0.f) {
		return INFINITY;
	}

	const float root = sqrtf(discriminant);
	const float t_enter = (-b - root) / a;

	if (t_enter < 0.f) {
		return INFINITY;
	}

	if (origin[2] + t_enter * direction[2] >= top) {
		return t_enter;
	}

	// enters the circle above the top, so it can only come down through the cap
	if (direction[2] > kDirectionEpsilon) {
		const float t_cap = (top - origin[2]) / direction[2];
		const float t_exit = (-b + root) / a;

		if (t_cap <= t_exit) {
			return t_cap;
		}
	}

	return INFINITY;
}

int World::boxCount() const
{
	return (_config.type == WorldType::Course) ? kCourseBoxes : 0;
}

const Box &World::box(int index) const
{
	return kCourse[index];
}

float World::nearestBox(const float point[3], int *index) const
{
	float best = INFINITY;

	for (int i = 0; i < boxCount(); i++) {
		float squared = 0.f;

		for (int axis = 0; axis < 3; axis++) {
			const float outside = fmaxf(fmaxf(kCourse[i].min[axis] - point[axis], point[axis] - kCourse[i].max[axis]), 0.f);
			squared += outside * outside;
		}

		const float distance = sqrtf(squared);

		if (distance < best) {
			best = distance;

			if (index) {
				*index = i;
			}
		}
	}

	return best;
}

float World::boxHit(const Box &box, const float origin[3], const float direction[3])
{
	// slab method: the ray is inside the box between the last entry and the first exit
	float t_enter = 0.f;
	float t_exit = INFINITY;

	for (int axis = 0; axis < 3; axis++) {
		if (fabsf(direction[axis]) < kDirectionEpsilon) {
			if (origin[axis] < box.min[axis] || origin[axis] > box.max[axis]) {
				return INFINITY;
			}

			continue;
		}

		float t_min = (box.min[axis] - origin[axis]) / direction[axis];
		float t_max = (box.max[axis] - origin[axis]) / direction[axis];

		if (t_min > t_max) {
			const float swap = t_min;
			t_min = t_max;
			t_max = swap;
		}

		t_enter = fmaxf(t_enter, t_min);
		t_exit = fminf(t_exit, t_max);
	}

	return (t_enter <= t_exit) ? t_enter : INFINITY;
}

float World::raycast(const float origin[3], const float direction[3], float max_range) const
{
	if (origin[2] > 0.f) {
		return 0.f;
	}

	float t_end = max_range;
	bool hit = false;

	if (direction[2] > kDirectionEpsilon) {
		const float t_ground = -origin[2] / direction[2];

		if (t_ground < t_end) {
			t_end = t_ground;
			hit = true;
		}
	}

	for (int i = 0; i < boxCount(); i++) {
		const float t = boxHit(kCourse[i], origin, direction);

		if (t <= t_end) {
			t_end = t;
			hit = true;
		}
	}

	if (_config.type == WorldType::Trees) {
		// 2D DDA over the cells under the ray, parameterised by distance along the 3D ray
		const float cell = _config.cell_size;
		int32_t cell_index[2] {cellIndex(origin[0]), cellIndex(origin[1])};
		int32_t step[2] {};
		float t_next[2] {};
		float t_delta[2] {};

		for (int axis = 0; axis < 2; axis++) {
			if (direction[axis] > kDirectionEpsilon) {
				step[axis] = 1;
				t_next[axis] = ((cell_index[axis] + 1) * cell - origin[axis]) / direction[axis];
				t_delta[axis] = cell / direction[axis];

			} else if (direction[axis] < -kDirectionEpsilon) {
				step[axis] = -1;
				t_next[axis] = (cell_index[axis] * cell - origin[axis]) / direction[axis];
				t_delta[axis] = -cell / direction[axis];

			} else {
				t_next[axis] = INFINITY;
				t_delta[axis] = INFINITY;
			}
		}

		// each step crosses one cell edge, a ray of length t_end crosses at most this many
		const int max_cells = 2 * (int)(t_end / cell) + 4;

		for (int i = 0; i < max_cells; i++) {
			Tree tree;

			if (treeInCell(cell_index[0], cell_index[1], tree)) {
				const float t = trunkHit(tree, origin, direction);

				// the trunk lies inside its cell, so the first hit along the ray is the nearest
				if (t <= t_end) {
					return t;
				}
			}

			const int axis = (t_next[0] < t_next[1]) ? 0 : 1;

			if (t_next[axis] > t_end) {
				break;
			}

			cell_index[axis] += step[axis];
			t_next[axis] += t_delta[axis];
		}
	}

	return hit ? t_end : INFINITY;
}

float World::nearestTrunk(float north, float east, float down, float search_radius, Tree *nearest) const
{
	float best = INFINITY;

	if (_config.type != WorldType::Trees) {
		return best;
	}

	const int32_t n_min = cellIndex(north - search_radius);
	const int32_t n_max = cellIndex(north + search_radius);
	const int32_t e_min = cellIndex(east - search_radius);
	const int32_t e_max = cellIndex(east + search_radius);

	for (int32_t cn = n_min; cn <= n_max; cn++) {
		for (int32_t ce = e_min; ce <= e_max; ce++) {
			Tree tree;

			if (!treeInCell(cn, ce, tree) || down < -tree.height) {
				continue;
			}

			const float distance = sqrtf((north - tree.north) * (north - tree.north)
						     + (east - tree.east) * (east - tree.east)) - tree.radius;

			if (distance <= search_radius && distance < best) {
				best = distance;

				if (nearest) {
					*nearest = tree;
				}
			}
		}
	}

	return best;
}

int World::treesNear(float north, float east, float radius, Tree *trees, int max_trees) const
{
	int count = 0;

	if (_config.type != WorldType::Trees) {
		return count;
	}

	const int32_t n_min = cellIndex(north - radius);
	const int32_t n_max = cellIndex(north + radius);
	const int32_t e_min = cellIndex(east - radius);
	const int32_t e_max = cellIndex(east + radius);

	for (int32_t cn = n_min; cn <= n_max; cn++) {
		for (int32_t ce = e_min; ce <= e_max; ce++) {
			Tree tree;

			if (count < max_trees && treeInCell(cn, ce, tree)
			    && (tree.north - north) * (tree.north - north) + (tree.east - east) * (tree.east - east) <= radius * radius) {
				trees[count++] = tree;
			}
		}
	}

	return count;
}

} // namespace obstacle_sim
