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

#include "ppk_stream.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanPpkStreamBridge::NAME = "ppk_stream";

UavcanPpkStreamBridge::UavcanPpkStreamBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_ppk_stream", ORB_ID(ppk_rtcm_data)),
	_sub_ppk_stream(node)
{
}

int
UavcanPpkStreamBridge::init()
{
	int res = _sub_ppk_stream.start(PPKStreamCbBinder(this, &UavcanPpkStreamBridge::ppk_stream_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanPpkStreamBridge::ppk_stream_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::gnss::PPKStream> &msg)
{
	ppk_rtcm_data_s ppk_data{};

	ppk_data.timestamp = hrt_absolute_time();

	// Copy data from UAVCAN message
	const size_t data_len = msg.data.size();
	ppk_data.len = (data_len <= sizeof(ppk_data.data)) ? data_len : sizeof(ppk_data.data);

	for (size_t i = 0; i < ppk_data.len; i++) {
		ppk_data.data[i] = msg.data[i];
	}

	publish(msg.getSrcNodeID().get(), &ppk_data);
}
