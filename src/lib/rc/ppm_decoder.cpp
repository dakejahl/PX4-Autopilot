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

#include "ppm_decoder.hpp"

void PPMDecoder::reset()
{
	*this = PPMDecoder{};
}

void PPMDecoder::resynchronize()
{
	// Deliberately leaves _last_edge alone. The next edge is then measured from the
	// last edge we trusted, which reads long and lands us back in the start-gap branch.
	_phase = Phase::UNSYNCH;
	_decoded_channels = 0;
}

void PPMDecoder::edge(uint64_t edge_time_us, bool edge_missed)
{
	if (edge_missed) {
		resynchronize();
		return;
	}

	const uint64_t width = edge_time_us - _last_edge;

	// A long gap ends the frame in progress and starts the next one
	if (width >= MIN_START) {
		if (_next_channel != _decoded_channels) {
			// The channel count changed. That may just be noise or a dropped edge, so
			// take a few consistent frames before believing it.
			if (_new_channel_count != _next_channel) {
				_new_channel_count = _next_channel;
				_new_channel_holdoff = CHANNEL_LOCK;

			} else if (_new_channel_holdoff > 0) {
				_new_channel_holdoff--;

			} else {
				_decoded_channels = _new_channel_count;
				_new_channel_count = 0;
			}

		} else if (_next_channel >= MIN_CHANNELS) {
			// Frame matches what we expect, publish it
			for (unsigned i = 0; i < _next_channel; i++) {
				_buffer[i] = _temp_buffer[i];
			}

			_last_valid_decode = edge_time_us;
		}

		_next_channel = 0;
		_phase = Phase::ARM;
		_last_edge = edge_time_us;
		return;
	}

	switch (_phase) {
	case Phase::UNSYNCH:
		// waiting for a start gap, nothing useful to do
		break;

	case Phase::ARM: {
			if (width < MIN_PULSE_WIDTH || width > MAX_PULSE_WIDTH) {
				resynchronize();
				return;
			}

			// first mark of the frame, expect an inactive edge next
			_last_mark = _last_edge;

			const uint64_t frame_length = _last_edge - _frame_start;
			_frame_length = (frame_length > UINT16_MAX) ? UINT16_MAX : (uint16_t)frame_length;
			_frame_start = _last_edge;
			_phase = Phase::ACTIVE;
			break;
		}

	case Phase::INACTIVE:
		if (width < MIN_PULSE_WIDTH || width > MAX_PULSE_WIDTH) {
			resynchronize();
			return;
		}

		// this edge carries no value, but the next mark does
		_phase = Phase::ACTIVE;
		break;

	case Phase::ACTIVE: {
			const uint64_t interval = edge_time_us - _last_mark;
			_last_mark = edge_time_us;

			if (interval < MIN_CHANNEL_VALUE || interval > MAX_CHANNEL_VALUE) {
				resynchronize();
				return;
			}

			if (_next_channel < MAX_CHANNELS) {
				_temp_buffer[_next_channel++] = (uint16_t)interval;
			}

			_phase = Phase::INACTIVE;
			break;
		}
	}

	_last_edge = edge_time_us;
}
