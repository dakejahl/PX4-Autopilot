/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>
#include <cmath>

namespace sensors
{

// Independent of the flight integrator: retain both endpoints and interpolate
// boundaries so each raw burst can be compared without sample-window rounding.
class FlowGyroBuffer
{
public:
	void reset() { _count = 0; _head = 0; }
	uint64_t newest() const { return _count ? at(_count - 1).time_us : 0; }

	void push(uint64_t time_us, const float rate[3], bool continuous)
	{
		if (!continuous || (_count && (time_us <= newest() || time_us - newest() > MAX_GAP_US))) {
			reset();
		}

		for (unsigned axis = 0; axis < 3; ++axis) {
			if (!std::isfinite(rate[axis])) {
				reset();
				return;
			}
		}

		_samples[_head] = {time_us, {rate[0], rate[1], rate[2]}};
		_head = (_head + 1) % CAPACITY;

		if (_count < CAPACITY) {
			++_count;
		}
	}

	bool integrate(uint64_t begin, uint64_t end, float integral[3], uint8_t &segments) const
	{
		segments = 0;
		integral[0] = integral[1] = integral[2] = NAN;

		if (_count < 2 || begin >= end || at(0).time_us > begin || newest() < end) {
			return false;
		}

		float sum[3] {};

		for (unsigned i = 1; i < _count; ++i) {
			const Sample &a = at(i - 1);
			const Sample &b = at(i);
			const uint64_t left = begin > a.time_us ? begin : a.time_us;
			const uint64_t right = end < b.time_us ? end : b.time_us;

			if (right <= left) {
				continue;
			}

			const float fraction = (float(left - a.time_us) + float(right - a.time_us)) / (b.time_us - a.time_us);

			for (unsigned axis = 0; axis < 3; ++axis) {
				sum[axis] += (a.rate[axis] + 0.5f * fraction * (b.rate[axis] - a.rate[axis])) * ((right - left) * 1e-6f);
			}

			++segments;
		}

		for (unsigned axis = 0; axis < 3; ++axis) {
			integral[axis] = sum[axis];
		}

		return true;
	}

private:
	static constexpr unsigned CAPACITY = 64;
	static constexpr uint64_t MAX_GAP_US = 10000;
	struct Sample {
		uint64_t time_us;
		float rate[3];
	};
	const Sample &at(unsigned index) const { return _samples[(_head + CAPACITY - _count + index) % CAPACITY]; }
	Sample _samples[CAPACITY] {};
	unsigned _head{0};
	unsigned _count{0};
};

} // namespace sensors
