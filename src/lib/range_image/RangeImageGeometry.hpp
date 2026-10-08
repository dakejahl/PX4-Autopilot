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
 * @file RangeImageGeometry.hpp
 *
 * Zone directions and range decoding for the range_image and range_image_info topics.
 *
 * No PX4 headers, so the log replay tool builds the same code on the host.
 */

#pragma once

#include <stdint.h>

namespace range_image
{

// Range counts, as in RangeImage.msg
static constexpr uint16_t kRangeNoReturn = 0;
static constexpr uint16_t kRangeMaxCount = 4093;
static constexpr uint16_t kRangeBelowMin = 4094;
static constexpr uint16_t kRangeInvalid = 4095;

// Enumerations, as in RangeImageInfo.msg
static constexpr uint8_t kProjectionAngular = 0;
static constexpr uint8_t kProjectionPinhole = 1;
static constexpr uint8_t kRangeTypeRadial = 0;
static constexpr uint8_t kRangeTypeDepth = 1;
static constexpr uint8_t kZoneOrderRowMajor = 0;
static constexpr uint8_t kZoneOrderColumnMajor = 1;

struct Geometry {
	uint16_t num_rows{0};
	uint16_t num_cols{0};
	uint8_t projection{kProjectionAngular};
	uint8_t range_type{kRangeTypeRadial};
	uint8_t zone_order{kZoneOrderRowMajor};
	float x_start{0.f};
	float x_step{0.f};
	float y_start{0.f};
	float y_step{0.f};
	const float *row_angle{nullptr};
	uint8_t row_angle_count{0};
};

/** Row and column of a zone, false if the zone is outside the grid. */
bool zoneRowCol(const Geometry &geometry, uint32_t zone, uint16_t &row, uint16_t &col);

/** Unit direction of a zone in the sensor frame (x forward, y right, z down). */
bool zoneDirection(const Geometry &geometry, uint32_t zone, float direction[3]);

/**
 * Distance along the zone direction for a range measured as range_type.
 * A depth is along sensor x, so it grows by 1 / direction x towards the edges.
 */
float radialRange(const Geometry &geometry, const float direction[3], float range);

} // namespace range_image
