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
#include <ardupilot/equipment/power/BatteryPeriodic.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/watts_battery_status.h>
#include <drivers/drv_hrt.h>

namespace uavcannode
{

class BatteryPeriodic :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ardupilot::equipment::power::BatteryPeriodic>
{
public:
	BatteryPeriodic(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::equipment::power::BatteryPeriodic::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(watts_battery_status)),
		uavcan::Publisher<ardupilot::equipment::power::BatteryPeriodic>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::equipment::power::BatteryPeriodic::getDataTypeFullName(),
			       ardupilot::equipment::power::BatteryPeriodic::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		static constexpr hrt_abstime BROADCAST_INTERVAL = 500000; // 2hz
		hrt_abstime now = hrt_absolute_time();

		if (now - _last_broadcast_time > BROADCAST_INTERVAL) {

			watts_battery_status_s status;
			if (uORB::SubscriptionCallbackWorkItem::update(&status)) {

				ardupilot::equipment::power::BatteryPeriodic battery = {};

				battery.design_capacity = 16000;
				battery.cycle_count = 123;
				battery.cells_in_series = status.cells_in_series;
				battery.cycle_count = status.cycle_count;
				battery.state_of_health = status.state_of_health;
				battery.design_capacity = status.design_capacity;
				battery.full_charge_capacity = status.actual_capacity;

				battery.name = "WATTS";
				battery.serial_number = "123456";
				battery.manufacture_date = "5_6_2022";

				uavcan::Publisher<ardupilot::equipment::power::BatteryPeriodic>::broadcast(battery);

				// ensure callback is registered
				uORB::SubscriptionCallbackWorkItem::registerCallback();
			}
		}
	}
private:
	hrt_abstime _last_broadcast_time{0};
};
} // namespace uavcan
