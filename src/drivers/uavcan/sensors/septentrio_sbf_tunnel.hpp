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

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/tunnel/Broadcast.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>

#include "sensor_bridge.hpp"

class UavcanSeptentrioSbfTunnelBridge : public IUavcanSensorBridge
{
public:
	static const char *const NAME;

	UavcanSeptentrioSbfTunnelBridge(uavcan::INode &node);
	~UavcanSeptentrioSbfTunnelBridge();

	const char *get_name() const override { return NAME; }

	int init() override;
	unsigned get_num_redundant_channels() const override { return 0; }
	void print_status() const override;
	void update() override;

private:
	void tunnel_sub_cb(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast> &msg);

	typedef uavcan::MethodBinder<UavcanSeptentrioSbfTunnelBridge *,
		void (UavcanSeptentrioSbfTunnelBridge::*)(const uavcan::ReceivedDataStructure<uavcan::tunnel::Broadcast> &)>
		TunnelCbBinder;

	uavcan::Subscriber<uavcan::tunnel::Broadcast, TunnelCbBinder> _sub_tunnel;

	void parse_sbf_stream();
	bool validate_sbf_crc(const uint8_t *data, uint16_t length);

	static const int STREAM_BUFFER_SIZE = 2048;
	uint8_t _stream_circular_buffer[STREAM_BUFFER_SIZE];
	uint8_t _stream_linear_buffer[STREAM_BUFFER_SIZE];

	uint16_t _stream_buffer_head{0};
	uint16_t _stream_buffer_tail{0};

	perf_counter_t _msg_received{perf_alloc(PC_COUNT, "sbf_tunnel: received")};
	perf_counter_t _meas_epoch_count{perf_alloc(PC_COUNT, "sbf_tunnel: MeasEpoch")};
	perf_counter_t _sat_visibility_count{perf_alloc(PC_COUNT, "sbf_tunnel: SatVisibility")};
	perf_counter_t _gps_nav_count{perf_alloc(PC_COUNT, "sbf_tunnel: GPSNav")};
	perf_counter_t _corrupted_count{perf_alloc(PC_COUNT, "sbf_tunnel: corrupted")};
	perf_counter_t _dropped_bytes{perf_alloc(PC_COUNT, "sbf_tunnel: dropped_bytes")};

	hrt_abstime _last_perf_log_time{0};
};
