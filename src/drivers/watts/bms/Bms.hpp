/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/geo/geo.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/watts_battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/shutdown.h>
#include <uORB/topics/button_pressed.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include "BQ76952.hpp"
#include "BQ34Z100.hpp"

using namespace time_literals;

class Bms : public ModuleBase<Bms>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Bms();
	~Bms() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init();

private:
	void Run() override;

	void update_params(const bool force = false);

	void handle_button_and_boot();
	void handle_idle_current_detection();
	void handle_automatic_protections();

	bool check_button_held();
	void shutdown();

	void collect_and_publish();

	int initialize_bq34();
	int initialize_bq76();

	// Shell commands
	int read_manu();
	int write_manu();
	int diagnostics();
	int flags();
	int on();
	int off();

private:
	static const hrt_abstime SAMPLE_INTERVAL{50_ms};

	BQ34Z100* _bq34{nullptr};
	BQ76952* _bq76{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<shutdown_s> _shutdown_pub{ORB_ID(shutdown)};
	uORB::Publication<button_pressed_s>	_button_pressed_pub{ORB_ID(button_pressed)};
	uORB::Publication<watts_battery_status_s> _battery_status_pub{ORB_ID(watts_battery_status)};

	perf_counter_t _cycle_perf{};

	// State variables
	hrt_abstime _pressed_start_time{0};
	bool _button_pressed{false};
	bool _booted{false};
	bool _booted_button_held{true};

	bool _below_idle_current{false};
	hrt_abstime _idle_start_time{0};

	bool _protections_enabled{true};

	bool _shutdown{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AUTO_PROTECT>)    _param_auto_protect,
		(ParamFloat<px4::params::PROTECT_CURRENT>)  _param_protect_current,
		(ParamInt<px4::params::IDLE_TIMEOUT>)    _param_idle_timeout,
		(ParamFloat<px4::params::IDLE_CURRENT>)    _param_idle_current,
		(ParamFloat<px4::params::PARALLEL_VOLTAGE>)    _param_parallel_voltage
	);
};
