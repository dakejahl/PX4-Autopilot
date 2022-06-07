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
/**
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alex@arkelectron.com>
 */

#pragma once

#include "sensor_bridge.hpp"
#include <uORB/topics/battery_status.h>
#include <ardupilot/equipment/power/BatteryContinuous.hpp>
#include <ardupilot/equipment/power/BatteryPeriodic.hpp>
#include <ardupilot/equipment/power/BatteryCells.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

class UavcanBatteryBridge : public UavcanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	UavcanBatteryBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void battery_continuous_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryContinuous> &msg);
	void battery_periodic_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryPeriodic> &msg);
	void battery_cells_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryCells> &msg);

	void sumDischarged(hrt_abstime timestamp, float current_a);
	uint8_t determineWarning(float remaining);

	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryContinuous> &) >
		BatteryContinuousCbBinder;

	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryPeriodic> &) >
		BatteryPeriodicCbBinder;

	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryCells> &) >
		BatteryCellsCbBinder;

	uavcan::Subscriber<ardupilot::equipment::power::BatteryContinuous, BatteryContinuousCbBinder> _sub_continuous;
	uavcan::Subscriber<ardupilot::equipment::power::BatteryPeriodic, BatteryPeriodicCbBinder> _sub_periodic;
	uavcan::Subscriber<ardupilot::equipment::power::BatteryCells, BatteryCellsCbBinder> _sub_cells;
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _param_bat_emergen_thr
	)

	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	battery_status_s _battery_status[battery_status_s::MAX_INSTANCES] {};
};
