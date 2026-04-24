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

const char *const UavcanFlowBridge::NAME = "flow";

UavcanFlowBridge::UavcanFlowBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_flow", ORB_ID(sensor_optical_flow), node_info_publisher),
	_sub_flow(node),
	_sub_flow_aux(node)
{
	set_device_type(DRV_FLOW_DEVTYPE_UAVCAN);
}

int
UavcanFlowBridge::init()
{
	int res = _sub_flow.start(FlowCbBinder(this, &UavcanFlowBridge::flow_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_flow_aux.start(FlowAuxCbBinder(this, &UavcanFlowBridge::flow_aux_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan flow aux sub: %d", res);
		return res;
	}

	return 0;
}

UavcanFlowBridge::AuxState *UavcanFlowBridge::aux_for_node(int node_id)
{
	for (auto &slot : _aux_cache) {
		if (slot.node_id == node_id) {
			return &slot;
		}
	}

	for (auto &slot : _aux_cache) {
		if (slot.node_id < 0) {
			slot.node_id = node_id;
			return &slot;
		}
	}

	return nullptr;
}

void UavcanFlowBridge::flow_aux_sub_cb(const uavcan::ReceivedDataStructure<com::ark::equipment::flow::MeasurementAux>
				       &msg)
{
	AuxState *aux = aux_for_node(msg.getSrcNodeID().get());

	if (aux == nullptr) {
		return;
	}

	// DSDL illumination_mode: 0=bright, 1=low, 2=super-low, 3=unknown.
	// uORB mode:              0=unknown, 1=bright, 2=low, 3=super-low.
	switch (msg.illumination_mode) {
	case 0:  aux->mode = sensor_optical_flow_s::MODE_BRIGHT;         break;

	case 1:  aux->mode = sensor_optical_flow_s::MODE_LOWLIGHT;       break;

	case 2:  aux->mode = sensor_optical_flow_s::MODE_SUPER_LOWLIGHT; break;

	default: aux->mode = sensor_optical_flow_s::MODE_UNKNOWN;        break;
	}

	aux->shutter             = msg.shutter;
	aux->motion              = msg.motion;
	aux->challenging_surface = msg.challenging_surface;
	aux->chip_health_ok      = msg.chip_health_ok;
	aux->discard_count       = msg.discard_count;
	aux->mode_change_count   = msg.mode_change_count;
}

void UavcanFlowBridge::flow_sub_cb(const uavcan::ReceivedDataStructure<com::hex::equipment::flow::Measurement> &msg)
{
	sensor_optical_flow_s flow{};
	flow.timestamp_sample = hrt_absolute_time(); // TODO

	flow.device_id = make_uavcan_device_id(msg);

	flow.pixel_flow[0] = msg.flow_integral[0];
	flow.pixel_flow[1] = msg.flow_integral[1];

	flow.integration_timespan_us = 1.e6f * msg.integration_interval; // s -> us

	flow.quality = msg.quality;

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

	const AuxState *aux = aux_for_node(msg.getSrcNodeID().get());

	if (aux != nullptr) {
		flow.mode                = aux->mode;
		flow.shutter             = aux->shutter;
		flow.motion              = aux->motion;
		flow.challenging_surface = aux->challenging_surface;
		flow.chip_health_ok      = aux->chip_health_ok;
		flow.discard_count       = aux->discard_count;
		flow.mode_change_count   = aux->mode_change_count;
	}

	flow.timestamp = hrt_absolute_time();

	publish(msg.getSrcNodeID().get(), &flow);

	// Register device capability if not already done
	if (_node_info_publisher != nullptr) {
		_node_info_publisher->registerDeviceCapability(msg.getSrcNodeID().get(),
				flow.device_id, NodeInfoPublisher::DeviceCapability::OPTICAL_FLOW);
	}
}
