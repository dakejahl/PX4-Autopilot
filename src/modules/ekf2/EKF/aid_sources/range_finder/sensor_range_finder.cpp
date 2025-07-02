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

/**
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include <aid_sources/range_finder/sensor_range_finder.hpp>

#include <lib/matrix/matrix/math.hpp>

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::setSample(const rangeSample &sample)
{
	_sample = sample;
}

bool SensorRangeFinder::timedOut(uint64_t time_now) const
{
	if (_sample.time_us > time_now) {
		return false;
	}

	// TODO: 200ms?
	return time_now > _sample.time_us + 200'000;
}

void SensorRangeFinder::setPitchOffset(float new_pitch_offset)
{
	if (fabsf(_pitch_offset_rad - new_pitch_offset) > FLT_EPSILON) {
		_sin_pitch_offset = sinf(new_pitch_offset);
		_cos_pitch_offset = cosf(new_pitch_offset);
		_pitch_offset_rad = new_pitch_offset;
	}
}

void SensorRangeFinder::setLimits(float min_distance, float max_distance)
{
	_rng_valid_min_val = min_distance;
	_rng_valid_max_val = max_distance;
}

void SensorRangeFinder::updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth)
{
	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

} // namespace sensor
} // namespace estimator
