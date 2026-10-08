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

#include "RangeImageGeometry.hpp"

#include <math.h>

using namespace range_image;

static constexpr float kTolerance = 1e-5f;

// the SIH default, a VL53L9CX binned 2x2: 27 columns over 55 deg, 21 rows over 42 deg, row 0 on top
static Geometry vl53l9Binned()
{
	Geometry geometry{};
	geometry.num_rows = 21;
	geometry.num_cols = 27;
	geometry.projection = kProjectionAngular;
	geometry.x_step = 55.f / 27.f;
	geometry.x_start = -27.5f + 0.5f * geometry.x_step;
	geometry.y_step = -2.f;
	geometry.y_start = 21.f - 1.f;
	return geometry;
}

TEST(RangeImageGeometry, CentreZoneLooksAlongTheBoresight)
{
	const Geometry geometry = vl53l9Binned();
	float direction[3];
	ASSERT_TRUE(zoneDirection(geometry, 10 * 27 + 13, direction));
	EXPECT_NEAR(direction[0], 1.f, kTolerance);
	EXPECT_NEAR(direction[1], 0.f, kTolerance);
	EXPECT_NEAR(direction[2], 0.f, kTolerance);
}

TEST(RangeImageGeometry, FirstZoneIsTopLeft)
{
	const Geometry geometry = vl53l9Binned();
	float direction[3];
	ASSERT_TRUE(zoneDirection(geometry, 0, direction));
	// left is negative y, up is negative z in FRD
	EXPECT_LT(direction[1], 0.f);
	EXPECT_LT(direction[2], 0.f);
	EXPECT_NEAR(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2], 1.f, kTolerance);
	EXPECT_NEAR(asinf(-direction[2]), 20.f * 0.0174533f, 1e-4f);
}

TEST(RangeImageGeometry, ColumnMajorOrder)
{
	Geometry geometry = vl53l9Binned();
	geometry.zone_order = kZoneOrderColumnMajor;
	uint16_t row = 0;
	uint16_t col = 0;
	ASSERT_TRUE(zoneRowCol(geometry, 21 * 3 + 5, row, col));
	EXPECT_EQ(row, 5);
	EXPECT_EQ(col, 3);
	EXPECT_FALSE(zoneRowCol(geometry, 21 * 27, row, col));
}

TEST(RangeImageGeometry, PinholeAndDepth)
{
	Geometry geometry{};
	geometry.num_rows = 1;
	geometry.num_cols = 3;
	geometry.projection = kProjectionPinhole;
	geometry.range_type = kRangeTypeDepth;
	geometry.x_start = -1.f;
	geometry.x_step = 1.f;

	float direction[3];
	ASSERT_TRUE(zoneDirection(geometry, 0, direction));
	// u = -1 is 45 deg left
	EXPECT_NEAR(direction[0], sqrtf(0.5f), kTolerance);
	EXPECT_NEAR(direction[1], -sqrtf(0.5f), kTolerance);
	// a wall 2 m ahead reads a depth of 2 m in every zone, 2 sqrt(2) m along the 45 deg ray
	EXPECT_NEAR(radialRange(geometry, direction, 2.f), 2.f * sqrtf(2.f), kTolerance);
}

TEST(RangeImageGeometry, RowTableReplacesTheStep)
{
	const float rows[2] {10.f, -30.f};
	Geometry geometry{};
	geometry.num_rows = 2;
	geometry.num_cols = 1;
	geometry.row_angle = rows;
	geometry.row_angle_count = 2;

	float direction[3];
	ASSERT_TRUE(zoneDirection(geometry, 1, direction));
	EXPECT_NEAR(asinf(-direction[2]), -30.f * 0.0174533f, 1e-4f);
}
