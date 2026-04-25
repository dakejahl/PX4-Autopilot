/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
#include <math.h>
#include "EKF/common.h"
#include "EKF/aid_sources/range_finder/sensor_range_finder.hpp"
#include <matrix/math.hpp>

using estimator::sensor::rangeSample;
using matrix::Dcmf;
using matrix::Eulerf;
using namespace estimator::sensor;

class SensorRangeFinderTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		_range_finder.setPitchOffset(0.f);
		_range_finder.setCosMaxTilt(0.707f);
	}

	void TearDown() override
	{
	}

protected:
	SensorRangeFinder _range_finder{};
	const rangeSample _good_sample{(uint64_t)2e6, 5.f, 100}; // {time_us, range, quality}

	void updateSensorAtRate(rangeSample sample, uint64_t duration_us, uint64_t dt_update_us, uint64_t dt_sensor_us);
	void testTilt(const Eulerf &euler, bool should_pass);
};

void SensorRangeFinderTest::updateSensorAtRate(rangeSample sample, uint64_t duration_us, uint64_t dt_update_us,
		uint64_t dt_sensor_us)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	rangeSample new_sample = sample;
	uint64_t t_now_us = sample.time_us;

	for (int i = 0; i < int(duration_us / dt_update_us); i++) {
		t_now_us += dt_update_us;

		if ((i % int(dt_sensor_us / dt_update_us)) == 0) {
			new_sample.time_us = t_now_us;
			_range_finder.setSample(new_sample);
		}

		_range_finder.runChecks(t_now_us, attitude);
	}
}

void SensorRangeFinderTest::testTilt(const Eulerf &euler, bool should_pass)
{
	const Dcmf attitude{euler};
	_range_finder.setSample(_good_sample);
	_range_finder.runChecks(_good_sample.time_us, attitude);

	if (should_pass) {
		EXPECT_TRUE(_range_finder.isDataHealthy());
		EXPECT_TRUE(_range_finder.isHealthy());

	} else {
		EXPECT_FALSE(_range_finder.isDataHealthy());
		EXPECT_FALSE(_range_finder.isHealthy());
	}
}

TEST_F(SensorRangeFinderTest, setRange)
{
	rangeSample sample{};
	sample.rng = 1.f;
	sample.time_us = 1e6;
	sample.quality = 9;

	_range_finder.setRange(sample.rng);
	_range_finder.setDataReadiness(true);
	_range_finder.setValidity(true);
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, goodData)
{
	// WHEN: the drone is leveled and the data is good
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};
	_range_finder.setSample(_good_sample);
	_range_finder.runChecks(_good_sample.time_us, attitude);

	// THEN: the data can be used for aiding
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, tiltExceeded)
{
	const Eulerf zero(0.f, 0.f, 0.f);
	const Eulerf pitch_46(0.f, 0.8f, 0.f);
	const Eulerf pitch_minus46(0.f, -0.8f, 0.f);
	const Eulerf pitch_40(0.f, 0.7f, 1.f);
	const Eulerf pitch_minus40(0.f, -0.7f, 0.f);
	const Eulerf roll_46(0.8f, 0.f, 0.f);
	const Eulerf roll_minus46(-0.8f, 0.f, 0.f);
	const Eulerf roll_40(0.7f, 0.f, 2.f);
	const Eulerf roll_minus40(-0.7f, 0.f, 3.f);
	const Eulerf roll_28_pitch_minus28(0.5f, -0.5f, 4.f);
	const Eulerf roll_46_pitch_minus46(0.8f, -0.8f, 4.f);

	testTilt(zero, true);
	testTilt(pitch_46, false);
	testTilt(pitch_minus46, false);
	testTilt(pitch_40, true);
	testTilt(pitch_minus40, true);
	testTilt(roll_46, false);
	testTilt(roll_minus46, false);
	testTilt(roll_40, true);
	testTilt(roll_minus40, true);
	testTilt(roll_28_pitch_minus28, true);
	testTilt(roll_46_pitch_minus46, false);

}

TEST_F(SensorRangeFinderTest, outOfDate)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data is outdated
	rangeSample outdated_sample = _good_sample;
	outdated_sample.time_us = 0;
	uint64_t t_now = _good_sample.time_us;
	_range_finder.setSample(outdated_sample);
	_range_finder.runChecks(t_now, attitude);

	// THEN: the data should be marked as unhealthy
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, badQuality)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: a sample arrives with quality == 0 (invalid signal per sensor)
	rangeSample bad = _good_sample;
	bad.quality = 0;
	_range_finder.setSample(bad);
	_range_finder.runChecks(bad.time_us, attitude);

	// THEN: the data is rejected
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	// AND WHEN: the next sample is good
	rangeSample good = _good_sample;
	good.time_us = bad.time_us + (uint64_t)1e5;
	_range_finder.setSample(good);
	_range_finder.runChecks(good.time_us, attitude);

	// THEN: it is accepted immediately (no hysteresis)
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, unknownQuality)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: a sample reports quality == -1 (unknown, e.g. analog sensor)
	rangeSample sample = _good_sample;
	sample.quality = -1;
	_range_finder.setSample(sample);
	_range_finder.runChecks(sample.time_us, attitude);

	// THEN: it is treated as healthy (only quality == 0 is rejected)
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, continuity)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};

	// WHEN: the data rate is too slow
	const uint64_t dt_update_us = 10e3;
	uint64_t dt_sensor_us = 4e6;
	uint64_t duration_us = 8e6;
	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: the data should be marked as unhealthy
	// Note that it also fails the out-of-date test here
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	// AND WHEN: the data rate is acceptable
	dt_sensor_us = 3e5;
	duration_us = 5e5;
	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);

	// THEN: it should still fail until the filter converge
	// to the new datarate
	EXPECT_FALSE(_range_finder.isDataHealthy());
	EXPECT_FALSE(_range_finder.isHealthy());

	updateSensorAtRate(_good_sample, duration_us, dt_update_us, dt_sensor_us);
	EXPECT_TRUE(_range_finder.isDataHealthy());
	EXPECT_TRUE(_range_finder.isHealthy());
}

TEST_F(SensorRangeFinderTest, distBottom)
{
	const Dcmf attitude{Eulerf(0.f, 0.f, 0.f)};
	rangeSample sample{};
	sample.rng = 1.f;
	sample.time_us = 1e6;
	sample.quality = 9;

	_range_finder.setSample(sample);
	_range_finder.runChecks(sample.time_us, attitude);
	EXPECT_FLOAT_EQ(_range_finder.getDistBottom(), sample.rng);

	const Dcmf attitude20{Eulerf(-0.35f, 0.f, 0.f)};
	_range_finder.runChecks(sample.time_us, attitude20);
	EXPECT_FLOAT_EQ(_range_finder.getDistBottom(), sample.rng * cosf(-0.35));
}
