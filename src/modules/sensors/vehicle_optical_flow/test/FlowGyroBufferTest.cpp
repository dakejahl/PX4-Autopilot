/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include "../FlowGyroBuffer.hpp"

using sensors::FlowGyroBuffer;

TEST(FlowGyroBuffer, InterpolatesBothBoundaries)
{
	FlowGyroBuffer buffer;

	for (unsigned t = 1000; t <= 5000; t += 1000) {
		const float rate[3] {float(t) * 1e-3f, -2.f, 0.f};
		buffer.push(t, rate, true);
	}

	float integral[3];
	uint8_t segments;
	ASSERT_TRUE(buffer.integrate(1500, 4250, integral, segments));
	EXPECT_NEAR(integral[0], 0.5f * (1.5f + 4.25f) * 0.00275f, 1e-8f);
	EXPECT_NEAR(integral[1], -0.0055f, 1e-8f);
	EXPECT_FLOAT_EQ(integral[2], 0.f);
	EXPECT_EQ(segments, 4);

	float first[3], second[3];
	ASSERT_TRUE(buffer.integrate(1500, 3000, first, segments));
	ASSERT_TRUE(buffer.integrate(3000, 4250, second, segments));
	EXPECT_NEAR(first[0] + second[0], integral[0], 1e-8f);
}

TEST(FlowGyroBuffer, MissingCoverageIsNotZeroMotion)
{
	FlowGyroBuffer buffer;
	const float rate[3] {1.f, 0.f, 0.f};
	buffer.push(1000, rate, true);
	buffer.push(2000, rate, true);
	float integral[3];
	uint8_t segments;

	for (const auto &window : {std::pair<uint64_t, uint64_t> {500, 2000}, {1000, 2500}, {2000, 1000}, {1000, 1000}}) {
		EXPECT_FALSE(buffer.integrate(window.first, window.second, integral, segments));
		EXPECT_TRUE(std::isnan(integral[0]));
		EXPECT_EQ(segments, 0);
	}
}

TEST(FlowGyroBuffer, QueueLossAndClockDiscontinuityInvalidateWindow)
{
	FlowGyroBuffer buffer;
	const float rate[3] {1.f, 2.f, 3.f};
	float integral[3];
	uint8_t segments;
	buffer.push(1000, rate, true);
	buffer.push(2000, rate, true);
	buffer.push(4000, rate, false);
	buffer.push(5000, rate, true);
	EXPECT_FALSE(buffer.integrate(1000, 5000, integral, segments));
	EXPECT_TRUE(buffer.integrate(4000, 5000, integral, segments));
	buffer.push(4500, rate, true);
	EXPECT_FALSE(buffer.integrate(4000, 5000, integral, segments));
	buffer.push(20000, rate, true);
	EXPECT_FALSE(buffer.integrate(4500, 20000, integral, segments));
	buffer.reset();
	EXPECT_EQ(buffer.newest(), 0);
}

TEST(FlowGyroBuffer, WrapAndNonfiniteInput)
{
	FlowGyroBuffer buffer;
	const float rate[3] {1.f, 2.f, 3.f};

	for (unsigned t = 1000; t <= 100000; t += 1000) {
		buffer.push(t, rate, true);
	}

	float integral[3];
	uint8_t segments;
	ASSERT_TRUE(buffer.integrate(90000, 100000, integral, segments));
	EXPECT_NEAR(integral[2], 0.03f, 1e-7f);
	EXPECT_FALSE(buffer.integrate(1000, 100000, integral, segments));
	const float bad_rate[3] {NAN, 0.f, 0.f};
	buffer.push(101000, bad_rate, true);
	EXPECT_FALSE(buffer.integrate(90000, 100000, integral, segments));
}
