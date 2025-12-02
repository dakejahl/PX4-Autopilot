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

#include <uavcan/equipment/gnss/PPKStream.hpp>

#include <lib/drivers/device/Device.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/ppk_rtcm_data.h>

namespace uavcannode
{

class PPKStreamPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::PPKStream>
{
public:
	PPKStreamPub(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::PPKStream::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(ppk_rtcm_data)),
		uavcan::Publisher<uavcan::equipment::gnss::PPKStream>(node)
	{
		this->setPriority(uavcan::TransferPriority::NumericallyMax);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::PPKStream::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::equipment::gnss::PPKStream;

		// ppk_rtcm_data -> uavcan::equipment::gnss::PPKStream
		ppk_rtcm_data_s ppk_data;

		if (uORB::SubscriptionCallbackWorkItem::update(&ppk_data)) {
			PPKStream ppk_stream{};
			ppk_stream.protocol_id = PPKStream::PROTOCOL_ID_RTCM3;

			const size_t capacity = ppk_stream.data.capacity();
			size_t written = 0;
			int result = 0;

			while ((result >= 0) && written < ppk_data.len) {
				size_t chunk_size = ppk_data.len - written;

				if (chunk_size > capacity) {
					chunk_size = capacity;
				}

				for (size_t i = 0; i < chunk_size; ++i) {
					ppk_stream.data.push_back(ppk_data.data[written]);
					written += 1;
				}

				result = uavcan::Publisher<PPKStream>::broadcast(ppk_stream);

				// ensure callback is registered
				uORB::SubscriptionCallbackWorkItem::registerCallback();

				ppk_stream.data.clear();
			}
		}
	}
};
} // namespace uavcannode
