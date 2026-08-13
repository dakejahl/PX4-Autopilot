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
 * @file ppm_decoder.hpp
 *
 * CPPM (PPM-sum) frame decoder.
 *
 * Feed it the timestamp of every signal edge and it recovers the channel
 * values. Nothing in here touches a timer or a register, so it works with any
 * edge source and can be exercised on the host.
 */

#pragma once

#include <stdint.h>

class PPMDecoder
{
public:
	static constexpr unsigned MIN_CHANNELS = 5;
	static constexpr unsigned MAX_CHANNELS = 20;

	/**
	 * Feed one signal edge.
	 *
	 * @param edge_time_us  edge timestamp in microseconds, monotonic
	 * @param edge_missed   true if the source knows it dropped an edge since the last call
	 */
	void edge(uint64_t edge_time_us, bool edge_missed = false);

	/** Reset to the state of a decoder that has never seen an edge. */
	void reset();

	unsigned channel_count() const { return _decoded_channels; }
	const uint16_t *channels() const { return _buffer; }
	uint64_t last_decode_time() const { return _last_valid_decode; }
	uint16_t frame_length() const { return _frame_length; }

private:
	/* decoder tuning, microseconds */
	static constexpr uint32_t MIN_PULSE_WIDTH = 200;	///< minimum width of a valid first pulse
	static constexpr uint32_t MAX_PULSE_WIDTH = 600;	///< maximum width of a valid first pulse
	static constexpr uint32_t MIN_CHANNEL_VALUE = 800;	///< shortest valid channel signal
	static constexpr uint32_t MAX_CHANNEL_VALUE = 2200;	///< longest valid channel signal
	static constexpr uint32_t MIN_START = 2300;		///< shortest valid start gap

	/** number of same-sized frames required to accept a new channel count */
	static constexpr unsigned CHANNEL_LOCK = 4;

	/** Abandon the frame in progress and wait for the next start gap. */
	void resynchronize();

	enum class Phase {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} _phase{Phase::UNSYNCH};

	uint64_t _last_edge{0};		///< last edge timestamp
	uint64_t _last_mark{0};		///< last significant edge timestamp
	uint64_t _frame_start{0};	///< start of the frame in progress
	uint64_t _last_valid_decode{0};

	unsigned _next_channel{0};
	unsigned _decoded_channels{0};
	uint16_t _frame_length{0};

	/* channel count settling, so noise cannot instantly redefine the frame */
	unsigned _new_channel_count{0};
	unsigned _new_channel_holdoff{0};

	uint16_t _buffer[MAX_CHANNELS] {};
	uint16_t _temp_buffer[MAX_CHANNELS] {};
};
