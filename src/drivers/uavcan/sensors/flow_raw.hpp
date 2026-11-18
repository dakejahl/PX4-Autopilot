/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "sensor_bridge.hpp"
#include <ark/flow/RawFlow.hpp>
#include <uORB/topics/flow_raw.h>

class UavcanRawFlowBridge : public UavcanSensorBridgeBase
{
public:
	UavcanRawFlowBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
		UavcanSensorBridgeBase("uavcan_flow_raw", ORB_ID(flow_raw), node_info_publisher),
		_sub(node)
	{
		set_device_type(DRV_FLOW_DEVTYPE_UAVCAN);
	}

	const char *get_name() const override { return "flow_raw"; }
	int init() override { return _sub.start(Callback(this, &UavcanRawFlowBridge::callback)); }

private:
	void callback(const uavcan::ReceivedDataStructure<ark::flow::RawFlow> &message)
	{
		flow_raw_s report{};
		const uint64_t bus_now = _sub.getNode().getUtcTime().toUSec();
		report.timestamp = hrt_absolute_time();
		report.timestamp_sample = uavcan_bridge::sample_timestamp(message.bus_timestamp_us, bus_now, report.timestamp,
					  &report.timestamp_sample_valid);
		report.bus_timestamp_us = message.bus_timestamp_us;
		report.device_id = make_uavcan_device_id(message);
		report.frame_counter = message.frame_counter;
		report.motion = message.motion;
		report.observation = message.observation;
		report.delta_x = message.delta_x;
		report.delta_y = message.delta_y;
		report.squal_raw = message.squal_raw;
		report.raw_data_sum = message.raw_data_sum;
		report.shutter = message.shutter;
		report.interval_us = message.interval_us;
		report.gyro_samples = message.gyro_samples;
		report.node_timestamp_us = message.node_timestamp_us;
		report.gyro_device_id = message.gyro_device_id;

		for (unsigned axis = 0; axis < 3; ++axis) {
			report.gyro_integral[axis] = message.gyro_integral[axis];
		}

		publish(message.getSrcNodeID().get(), &report);
	}

	using Callback = uavcan::MethodBinder<UavcanRawFlowBridge *,
	      void (UavcanRawFlowBridge::*)(const uavcan::ReceivedDataStructure<ark::flow::RawFlow> &)>;
	uavcan::Subscriber<ark::flow::RawFlow, Callback> _sub;
};
