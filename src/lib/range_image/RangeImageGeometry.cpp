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

#include "RangeImageGeometry.hpp"

#include <math.h>

namespace range_image
{

static constexpr float kDegToRad = 0.017453292519943295f;

// a depth this close to perpendicular to the boresight cannot be turned into a radial range
static constexpr float kMinBoresightCosine = 1e-3f;

bool zoneRowCol(const Geometry &geometry, uint32_t zone, uint16_t &row, uint16_t &col)
{
	if (geometry.num_rows == 0 || geometry.num_cols == 0
	    || zone >= (uint32_t)geometry.num_rows * geometry.num_cols) {
		return false;
	}

	if (geometry.zone_order == kZoneOrderColumnMajor) {
		col = zone / geometry.num_rows;
		row = zone % geometry.num_rows;

	} else {
		row = zone / geometry.num_cols;
		col = zone % geometry.num_cols;
	}

	return true;
}

bool zoneDirection(const Geometry &geometry, uint32_t zone, float direction[3])
{
	uint16_t row = 0;
	uint16_t col = 0;

	if (!zoneRowCol(geometry, zone, row, col)) {
		return false;
	}

	const float x = geometry.x_start + col * geometry.x_step;
	const float y = (row < geometry.row_angle_count && geometry.row_angle != nullptr) ? geometry.row_angle[row]
			: geometry.y_start + row * geometry.y_step;

	if (geometry.projection == kProjectionPinhole) {
		const float norm = sqrtf(1.f + x * x + y * y);
		direction[0] = 1.f / norm;
		direction[1] = x / norm;
		direction[2] = -y / norm;

	} else {
		const float azimuth = x * kDegToRad;
		const float elevation = y * kDegToRad;
		direction[0] = cosf(elevation) * cosf(azimuth);
		direction[1] = cosf(elevation) * sinf(azimuth);
		direction[2] = -sinf(elevation);
	}

	return true;
}

float radialRange(const Geometry &geometry, const float direction[3], float range)
{
	if (geometry.range_type == kRangeTypeDepth) {
		return (direction[0] > kMinBoresightCosine) ? range / direction[0] : NAN;
	}

	return range;
}

} // namespace range_image
