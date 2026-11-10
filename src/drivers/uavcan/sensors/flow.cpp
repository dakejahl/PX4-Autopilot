/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "flow.hpp"

#include <drivers/drv_hrt.h>
#include <parameters/param.h>

const char *const UavcanFlowBridge::NAME = "flow";

UavcanFlowBridge::UavcanFlowBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_flow", ORB_ID(sensor_optical_flow), node_info_publisher),
	_sub_flow(node)
{
	set_device_type(DRV_FLOW_DEVTYPE_UAVCAN);
}

int
UavcanFlowBridge::init()
{
	param_get(param_find("UAVCAN_FLOW_CHAL"), &_challenging_surface_mode);

	int res = _sub_flow.start(FlowCbBinder(this, &UavcanFlowBridge::flow_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanFlowBridge::flow_sub_cb(const uavcan::ReceivedDataStructure<com::hex::equipment::flow::Measurement> &msg)
{
	sensor_optical_flow_s flow{};
	flow.timestamp_sample = hrt_absolute_time(); // TODO

	flow.device_id = make_uavcan_device_id(msg);

	flow.pixel_flow[0] = msg.flow_integral[0];
	flow.pixel_flow[1] = msg.flow_integral[1];

	flow.integration_timespan_us = 1.e6f * msg.integration_interval; // s -> us

	if (_challenging_surface_mode > 0) {
		// Only nodes built to pack set bit 0. Others send raw quality there, so decoding is opt-in.
		flow.challenging_surface = msg.quality & 1u;
		flow.quality = msg.quality & static_cast<uint8_t>(~1u);

		if (_challenging_surface_mode == 2 && flow.challenging_surface) {
			flow.quality = 0;
		}

		// These nodes also report the light their sensor works in through their NodeStatus vendor code, since the flow
		// message has no field for it: bits 1:0 the light mode, bit 6 the shutter being at the mode's limit. It arrives
		// at 10 Hz, so it can trail a change by a tenth of a second.
		for (auto &node_status_sub : _node_status_subs) {
			dronecan_node_status_s node_status;

			if (node_status_sub.copy(&node_status) && (node_status.node_id == msg.getSrcNodeID().get())) {
				flow.mode = node_status.vendor_specific_status_code & 0x3;
				flow.shutter_at_limit = (node_status.vendor_specific_status_code >> 6) & 0x1;
				break;
			}
		}

	} else {
		flow.quality = msg.quality;
	}

	if (PX4_ISFINITE(msg.rate_gyro_integral[0]) && PX4_ISFINITE(msg.rate_gyro_integral[1])) {
		flow.delta_angle[0] = msg.rate_gyro_integral[0];
		flow.delta_angle[1] = msg.rate_gyro_integral[1];
		flow.delta_angle[2] = NAN;
		flow.delta_angle_available = true;

	} else {
		flow.delta_angle[0] = NAN;
		flow.delta_angle[1] = NAN;
		flow.delta_angle[2] = NAN;
	}

	flow.max_flow_rate = NAN;
	flow.min_ground_distance = NAN;
	flow.max_ground_distance = NAN;

	flow.timestamp = hrt_absolute_time();

	publish(msg.getSrcNodeID().get(), &flow);

	// Register device capability if not already done
	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				flow.device_id, device_information_s::DEVICE_TYPE_OPTICAL_FLOW);
	}
}
