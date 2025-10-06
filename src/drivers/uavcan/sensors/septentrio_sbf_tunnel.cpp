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
 *    the distribution and/or other materials provided with the
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

#include "septentrio_sbf_tunnel.hpp"
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

// SBF constants
static constexpr uint8_t SBF_SYNC1 = 0x24; // '$'
static constexpr uint8_t SBF_SYNC2 = 0x40; // '@'
static constexpr uint16_t SBF_BLOCK_MEAS_EPOCH = 4027;
static constexpr uint16_t SBF_BLOCK_SAT_VISIBILITY = 4012;
static constexpr uint16_t SBF_BLOCK_GPS_NAV = 5891;

const char *const UavcanSeptentrioSbfTunnelBridge::NAME = "septentrio_sbf_tunnel";

UavcanSeptentrioSbfTunnelBridge::UavcanSeptentrioSbfTunnelBridge(uavcan::INode &node) :
	_sub_tunnel(node)
{
}

UavcanSeptentrioSbfTunnelBridge::~UavcanSeptentrioSbfTunnelBridge()
{
	perf_free(_msg_received);
	perf_free(_meas_epoch_count);
	perf_free(_sat_visibility_count);
	perf_free(_gps_nav_count);
	perf_free(_corrupted_count);
	perf_free(_dropped_bytes);
}

int UavcanSeptentrioSbfTunnelBridge::init()
{
	int res = _sub_tunnel.start(TunnelCbBinder(this, &UavcanSeptentrioSbfTunnelBridge::tunnel_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start tunnel subscriber");
		return res;
	}

	return 0;
}

void UavcanSeptentrioSbfTunnelBridge::tunnel_sub_cb(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast>
		&msg)
{
	// Only process GPS_GENERIC protocol messages
	if (msg.protocol.protocol != msg.protocol.GPS_GENERIC) {
		return;
	}

	perf_count(_msg_received);

	// Append received data to circular stream buffer
	for (unsigned i = 0; i < msg.buffer.size(); i++) {
		uint16_t next_head = (_stream_buffer_head + 1) % STREAM_BUFFER_SIZE;

		if (next_head != _stream_buffer_tail) {
			_stream_circular_buffer[_stream_buffer_head] = msg.buffer[i];
			_stream_buffer_head = next_head;

		} else {
			// Buffer full
			PX4_WARN("SBF tunnel stream buffer overflow");
			break;
		}
	}
}

void UavcanSeptentrioSbfTunnelBridge::update()
{
	// Parse any complete SBF messages in the stream buffer
	parse_sbf_stream();

	// Log perf counters every 5 seconds
	hrt_abstime now = hrt_absolute_time();

	if (_last_perf_log_time == 0) {
		_last_perf_log_time = now;

	} else if (hrt_elapsed_time(&_last_perf_log_time) >= 5000000) { // 5 seconds
		PX4_INFO("SBF tunnel: MeasEpoch=%llu SatVisibility=%llu GPSNav=%llu Corrupted=%llu DroppedBytes=%llu",
			 perf_event_count(_meas_epoch_count),
			 perf_event_count(_sat_visibility_count),
			 perf_event_count(_gps_nav_count),
			 perf_event_count(_corrupted_count),
			 perf_event_count(_dropped_bytes));
		_last_perf_log_time = now;
	}
}

void UavcanSeptentrioSbfTunnelBridge::parse_sbf_stream()
{
	while (true) {
		// Check if we have at least the SBF header (8 bytes)
		uint16_t bytes_available;

		if (_stream_buffer_head >= _stream_buffer_tail) {
			bytes_available = _stream_buffer_head - _stream_buffer_tail;

		} else {
			bytes_available = STREAM_BUFFER_SIZE - _stream_buffer_tail + _stream_buffer_head;
		}

		if (bytes_available < 8) {
			return; // Not enough data for header
		}

		// Look for sync bytes
		uint16_t idx = _stream_buffer_tail;
		uint8_t sync1 = _stream_circular_buffer[idx];

		if (sync1 != SBF_SYNC1) {
			// Discard byte and continue
			_stream_buffer_tail = (_stream_buffer_tail + 1) % STREAM_BUFFER_SIZE;
			perf_count(_dropped_bytes);
			continue;
		}

		uint8_t sync2 = _stream_circular_buffer[(idx + 1) % STREAM_BUFFER_SIZE];

		if (sync2 != SBF_SYNC2) {
			// Discard byte and continue
			_stream_buffer_tail = (_stream_buffer_tail + 1) % STREAM_BUFFER_SIZE;
			perf_count(_dropped_bytes);
			continue;
		}

		// Read length field (bytes 6-7)
		uint16_t length = _stream_circular_buffer[(idx + 6) % STREAM_BUFFER_SIZE] |
				  (_stream_circular_buffer[(idx + 7) % STREAM_BUFFER_SIZE] << 8);

		// Validate length is reasonable (4 to 2048 bytes)
		if (length < 8 || length > 2048 || (length % 4) != 0) {
			_stream_buffer_tail = (_stream_buffer_tail + 1) % STREAM_BUFFER_SIZE;
			perf_count(_corrupted_count);
			continue;
		}

		// Check if we have the complete message
		if (bytes_available < length) {
			return; // Wait for more data
		}

		// Extract the full message into a linear buffer
		for (uint16_t i = 0; i < length; i++) {
			_stream_linear_buffer[i] = _stream_circular_buffer[(_stream_buffer_tail + i) % STREAM_BUFFER_SIZE];
		}

		// Validate CRC
		if (!validate_sbf_crc(_stream_linear_buffer, length)) {
			_stream_buffer_tail = (_stream_buffer_tail + 1) % STREAM_BUFFER_SIZE;
			perf_count(_corrupted_count);
			continue;
		}

		// Extract block ID (bytes 4-5, lower 13 bits)
		uint16_t block_id = (_stream_linear_buffer[4] | (_stream_linear_buffer[5] << 8)) & 0x1FFF;

		// TODO: Handle the data here. Parse out the SBF sub-blocks and publish data of interest to uORB.

		// LOGGING
		if (block_id == SBF_BLOCK_MEAS_EPOCH) {
			perf_count(_meas_epoch_count);

		} else if (block_id == SBF_BLOCK_SAT_VISIBILITY) {
			perf_count(_sat_visibility_count);

		} else if (block_id == SBF_BLOCK_GPS_NAV) {
			perf_count(_gps_nav_count);
		}

		// Advance tail past this message
		_stream_buffer_tail = (_stream_buffer_tail + length) % STREAM_BUFFER_SIZE;
	}
}

bool UavcanSeptentrioSbfTunnelBridge::validate_sbf_crc(const uint8_t *data, uint16_t length)
{
	// CRC-CCITT (0xFFFF) with polynomial 0x1021
	// Covers from ID field (byte 4) to end of message
	uint16_t crc = 0;

	for (uint16_t i = 4; i < length; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	// CRC is stored in bytes 2-3 (little endian)
	uint16_t stored_crc = data[2] | (data[3] << 8);

	return crc == stored_crc;
}

void UavcanSeptentrioSbfTunnelBridge::print_status() const
{
	PX4_INFO("tunnel chunks received: %llu", perf_event_count(_msg_received));
	PX4_INFO("MeasEpoch: %llu, SatVisibility: %llu, GPSNav: %llu",
		 perf_event_count(_meas_epoch_count),
		 perf_event_count(_sat_visibility_count),
		 perf_event_count(_gps_nav_count));
	PX4_INFO("Corrupted messages: %llu, Dropped bytes: %llu",
		 perf_event_count(_corrupted_count),
		 perf_event_count(_dropped_bytes));
	PX4_INFO("stream buffer usage: %u/%u bytes",
		 (_stream_buffer_head >= _stream_buffer_tail) ?
		 (_stream_buffer_head - _stream_buffer_tail) :
		 (STREAM_BUFFER_SIZE - _stream_buffer_tail + _stream_buffer_head),
		 STREAM_BUFFER_SIZE);
}
