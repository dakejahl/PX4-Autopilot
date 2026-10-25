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

#include "UavcanPublisherBase.hpp"
#include <ark/flow/RawFlow.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/flow_raw.h>

namespace uavcannode
{

class RawFlow :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ark::flow::RawFlow>
{
public:
	RawFlow(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ark::flow::RawFlow::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(flow_raw)),
		uavcan::Publisher<ark::flow::RawFlow>(node)
	{
		setPriority(uavcan::TransferPriority::Lowest);
		setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
	}

	void PrintInfo() override
	{
		printf("\tflow_raw -> %s:%d, broadcast failures: %lu\n",
		       ark::flow::RawFlow::getDataTypeFullName(), ark::flow::RawFlow::DefaultDataTypeID,
		       (unsigned long)_broadcast_failures);
	}

	void BroadcastAnyUpdates() override
	{
		flow_raw_s report;

		for (unsigned i = 0; i < flow_raw_s::ORB_QUEUE_LENGTH && uORB::SubscriptionCallbackWorkItem::update(&report); ++i) {
			ark::flow::RawFlow message{};
			message.frame_counter = report.frame_counter;
			message.motion = report.motion;
			message.observation = report.observation;
			message.delta_x = report.delta_x;
			message.delta_y = report.delta_y;
			message.squal_raw = report.squal_raw;
			message.raw_data_sum = report.raw_data_sum;
			message.shutter = report.shutter;
			message.interval_us = report.interval_us;
			message.gyro_samples = report.gyro_samples;
			message.node_timestamp_us = report.node_timestamp_us;
			message.gyro_device_id = report.gyro_device_id;
			message.bus_timestamp_us = bus_timestamp_usec(getNode(), report.timestamp_sample);

			for (unsigned axis = 0; axis < 3; ++axis) {
				message.gyro_integral[axis] = report.gyro_integral[axis];
			}

			if (broadcast(message) < 0) {
				++_broadcast_failures;
			}

			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}

private:
	uint32_t _broadcast_failures{0};
};

} // namespace uavcannode
