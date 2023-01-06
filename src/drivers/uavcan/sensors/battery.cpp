/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "battery.hpp"

#include <lib/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_continuous(node),
	_sub_periodic(node),
	_sub_cells(node)
{
}

int UavcanBatteryBridge::init()
{
	int res = _sub_continuous.start(BatteryContinuousCbBinder(this, &UavcanBatteryBridge::battery_continuous_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_periodic.start(BatteryPeriodicCbBinder(this, &UavcanBatteryBridge::battery_periodic_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_cells.start(BatteryCellsCbBinder(this, &UavcanBatteryBridge::battery_cells_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBatteryBridge::battery_continuous_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryContinuous> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get() || _battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].id = msg.getSrcNodeID().get();
	_battery_status[instance].connected = true; // TODO: timeout

	_battery_status[instance].voltage_v = msg.voltage;
	_battery_status[instance].voltage_filtered_v = msg.voltage;
	_battery_status[instance].current_a = msg.current;
	_battery_status[instance].current_filtered_a = msg.current;
	// _battery_status[instance].capacity = msg.full_charge_capacity;
	_battery_status[instance].remaining = msg.state_of_charge;
	_battery_status[instance].discharged_mah = msg.capacity_consumed;
	_battery_status[instance].custom_faults = msg.status_flags;


	// _battery_status[instance].scale = msg.; // Power scaling factor, >= 1, or -1 if unknown
	_battery_status[instance].temperature = msg.temperature_pcb;
	// _battery_status[instance].cell_count = msg.;
	_battery_status[instance].connected = true;
	_battery_status[instance].source = battery_status_s::BATTERY_SOURCE_EXTERNAL;
	// _battery_status[instance].priority = msg.;
	// _battery_status[instance].time_remaining_s = msg.;
	// _battery_status[instance].average_time_to_empty = msg.;
	// _battery_status[instance].serial_number = msg.model_instance_id;
	_battery_status[instance].warning = determineWarning(_battery_status[instance].remaining);

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);
}

void
UavcanBatteryBridge::battery_periodic_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryPeriodic> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get() || _battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].id = msg.getSrcNodeID().get();

	_battery_status[instance].design_capacity = msg.design_capacity;
	_battery_status[instance].cycle_count = msg.cycle_count;
	_battery_status[instance].state_of_health = msg.state_of_health_pct;
	_battery_status[instance].cell_count = msg.cells_in_series;
	_battery_status[instance].capacity = msg.full_charge_capacity;

	// memcpy(_battery_status[instance].serial_number, msg.serial_number , 2); // We only use the first 2 bytes
	// name
	// manufacture_date // formated as: Day + Month×32 + (Year–1980)×512
	// _battery_status[instance].serial_number = (uint16_t)msg.serial_number; // TODO:

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);
}

void
UavcanBatteryBridge::battery_cells_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryCells> &msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (_battery_status[instance].id == msg.getSrcNodeID().get() || _battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	_battery_status[instance].timestamp = hrt_absolute_time();
	_battery_status[instance].id = msg.getSrcNodeID().get();

	memcpy(_battery_status[instance].voltage_cell_v, &msg.voltages[0], msg.voltages.size() * sizeof(msg.voltages[0]));

	publish(msg.getSrcNodeID().get(), &_battery_status[instance]);
}

uint8_t
UavcanBatteryBridge::determineWarning(float remaining)
{
	uint8_t warning = {};
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining < _param_bat_emergen_thr.get()) {
		warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (remaining < _param_bat_crit_thr.get()) {
		warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (remaining < _param_bat_low_thr.get()) {
		warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	return warning;
}
