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
#include <ardupilot/equipment/power/BatteryContinuous.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/watts_battery_status.h>
#include <uORB/topics/watts_battery_status.h>
#include <drivers/drv_hrt.h>

namespace uavcannode
{

class BatteryContinuous :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ardupilot::equipment::power::BatteryContinuous>
{
public:
	BatteryContinuous(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::equipment::power::BatteryContinuous::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(watts_battery_status)),
		uavcan::Publisher<ardupilot::equipment::power::BatteryContinuous>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::equipment::power::BatteryContinuous::getDataTypeFullName(),
			       ardupilot::equipment::power::BatteryContinuous::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		static constexpr hrt_abstime BROADCAST_INTERVAL = 100000; // 10hz
		hrt_abstime now = hrt_absolute_time();

		if (now - _last_broadcast_time > BROADCAST_INTERVAL) {

			watts_battery_status_s status;
			if (uORB::SubscriptionCallbackWorkItem::update(&status)) {

				ardupilot::equipment::power::BatteryContinuous battery = {};
				battery.temperature_cells = status.temperature_cells;
				battery.temperature_pcb = status.temperature_pcb;
				battery.temperature_other = status.temperature_other;
				battery.current = status.current;
				battery.voltage = status.voltage;

				battery.state_of_charge = status.state_of_charge;
				battery.slot_id = 69; // TODO: how should we use this?

				battery.capacity_consumed = status.actual_capacity - status.capacity_remaining;
				battery.status_flags = status.status_flags; // uORB <--> DroneCAN <--> Mavlink BatteryV2

				// if (battery.current > 0.0f) {
				// 	battery.status = ardupilot::equipment::power::BatteryContinuous::STATUS_FLAG_CHARGING;

				// } else {
				// 	battery.status = ardupilot::equipment::power::BatteryContinuous::STATUS_FLAG_IN_USE;
				// }

				uavcan::Publisher<ardupilot::equipment::power::BatteryContinuous>::broadcast(battery);

				_last_broadcast_time = now;

				// ensure callback is registered
				uORB::SubscriptionCallbackWorkItem::registerCallback();
			}
		}
	}
private:
	hrt_abstime _last_broadcast_time{0};
};
} // namespace uavcan
