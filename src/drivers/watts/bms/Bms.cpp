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

// The Texas Instruments BQ76952 is a highly integrated, high accuracy battery monitor and protector for 3-series
// to 16-series li-ion, li-polymer, and LiFePO4 battery packs. The device includes a high accuracy monitoring
// system, a highly configurable protection subsystem, and support for autonomous or host controlled cell
// balancing.

#include "Bms.hpp"

static constexpr hrt_abstime SAMPLE_INTERVAL = 50_ms;

Bms::Bms() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::I2C1),
	_cycle_perf(perf_alloc(PC_ELAPSED, "Bms: single-sample"))
{
	_battery_status_pub.advertise();

	_current_filter.setParameters(SAMPLE_INTERVAL, 50_ms);
}

Bms::~Bms()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	_battery_status_pub.unadvertise();
	if (_bq34) delete _bq34;
	if (_bq76) delete _bq76;
}

int Bms::init()
{
	PX4_INFO("Initializing BMS");

	update_params(true);

	///// Write settings -- these need to made defaults that are baked into ASIC /////
	if (initialize_bq76() != PX4_OK) {
		return PX4_ERROR;
	}

	if (initialize_bq34() != PX4_OK) {
		return PX4_ERROR;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int Bms::initialize_bq76()
{
	_bq76 = new BQ76952();

	if (!_bq76) {
		PX4_INFO("failed to create BQ76952");
		return PX4_ERROR;
	}

	if (_bq76->init() != PX4_OK) {
		PX4_INFO("failed to initialize BQ76952");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int Bms::initialize_bq34()
{
	_bq34 = new BQ34Z100();

	if (!_bq34) {
		PX4_INFO("failed to create BQ34Z100");
		return PX4_ERROR;
	}

	if (_bq34->init() != PX4_OK) {
		PX4_INFO("failed to initialize BQ34Z100");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void Bms::Run()
{
	if (should_exit()) {
		PX4_INFO("exiting");
		exit_and_cleanup();
		return;
	}

	if (_shutdown) {
		// Wait until button is released
		if (px4_arch_gpioread(GPIO_N_BTN)) {
			shutdown();
		}
		return;
	}

	perf_begin(_cycle_perf);

	update_params();

	// Detect button hold and press and performs behaviors (bootup/shutdown/toggle-page)
	// and also publishes the state (BOOTING, BOOTED, SHUTDOWN)
	if (!_booted) {
		handle_button_booting();
	} else {
		handle_button_running();
	}
	// handle_button_behaviors();

	// If drawing a very low amount of power for some amount of time, automatically turn pack off
	// handle_idle_current_detection();

	collect_and_publish();

	perf_end(_cycle_perf);
}

void Bms::collect_and_publish()
{
	watts_battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	// BQ76
	battery_status.temperature_cells = 		_bq76->temperature_cells();
	battery_status.temperature_pcb = 		_bq76->temperature_fets();
	battery_status.temperature_other = 		NAN; // unused
	battery_status.current = 				_bq76->current();
	battery_status.current_filtered = 		_current_filter.update(battery_status.current);
	battery_status.voltage = 				_bq76->bat_voltage();
	_bq76->cell_voltages(battery_status.cell_voltages, 12);
	battery_status.status_flags = 			_bq76->status_flags();

	// BQ34
	battery_status.state_of_charge = 		_bq34->data_command(BQ34Z100::StateOfCharge); // 0 - 100
	battery_status.capacity_remaining = 	_bq34->data_command(BQ34Z100::RemainingCapacity) * _param_capacity_scalar.get(); // mAh
	battery_status.design_capacity = 		_bq34->data_command(BQ34Z100::DesignCapacity) * _param_capacity_scalar.get(); // mAh
	battery_status.actual_capacity = 		_bq34->data_command(BQ34Z100::FullChargeCapacity) * _param_capacity_scalar.get(); // mAh
	battery_status.cycle_count = 			_bq34->data_command(BQ34Z100::CycleCount);
	battery_status.state_of_health = 		_bq34->data_command(BQ34Z100::StateOfHealth); // 0 - 100
	battery_status.cells_in_series = 		12;

	_battery_status_pub.publish(battery_status);
}

void Bms::handle_idle_current_detection()
{
	int32_t idle_timeout = _param_idle_timeout.get();
	if (idle_timeout == 0) {
		return;
	}

	float current = _bq76->current();
	hrt_abstime now = hrt_absolute_time();

	if (current < _param_idle_current.get()) {
		if (!_below_idle_current) {
			_below_idle_current = true;
			_idle_start_time = now;
		}

		hrt_abstime timeout = (hrt_abstime)idle_timeout * 1e6; // us --> s
		if (now > _idle_start_time + timeout) {
			PX4_INFO("Battery has been idle for %llu, shutting down", timeout);
			app_state_s state = {};
			state.state = app_state_s::SHUTDOWN;
			_app_state_pub.publish(state);
			_shutdown = true;
		}

	} else {
		_below_idle_current = false;
	}
}

void Bms::handle_button_booting()
{
	// Button held
	if (check_button_held()) {
		PX4_INFO("Button was held, enabling FETs");
		_booted = true;
		_booted_button_held = true;
		_bq76->enable_fets();
	}

	// Check if PACK voltage is high
	float pack_voltage = _bq76->pack_voltage();
	if (pack_voltage >= _param_parallel_voltage.get()) {
		PX4_INFO("PACK voltage (%fv) above threshold (%fv), enabling FETs", double(pack_voltage), double(_param_parallel_voltage.get()));
		_booted = true;
		_bq76->enable_fets();
		return;
	}

	hrt_abstime now = hrt_absolute_time();
	// If button is being held update boot progress
	if (!px4_arch_gpioread(GPIO_N_BTN)) {
		hrt_abstime duration = now - _pressed_start_time;
		uint8_t progress = (100 * duration) / BUTTON_HOLD_TIME;
		if (progress > 100) {
			progress = 100;
		}
		app_state_s state = {};
		state.state = app_state_s::BOOTING;
		state.progress = progress;
		_app_state_pub.publish(state);
	} else {
		// Button released, show battery info and power off
		app_state_s state = {};
		state.state = app_state_s::SHOW_INFO;
		_app_state_pub.publish(state);
	}

	// Check if time since last press has elapsed 3 seconds
	hrt_abstime elapsed = now - _pressed_start_time;
	if (elapsed > BUTTON_SHUTDOWN_TIME) {
		PX4_INFO("Button not held, shutting down");
		app_state_s state = {};
		state.state = app_state_s::SHUTDOWN;
		_app_state_pub.publish(state);
		_shutdown = true;
	}
}

void Bms::handle_button_running()
{
	if (_booted_button_held) {
		// We must make sure the button is released before we start monitoring for shutdown / screen toggle
		if (px4_arch_gpioread(GPIO_N_BTN)) {
			PX4_INFO("Button released");
			_booted_button_held = false;
		}

	} else {
		if (check_button_held()) {
			PX4_INFO("Button was held, shutting down");
			app_state_s state = {};
			state.state = app_state_s::SHUTDOWN;
			_app_state_pub.publish(state);
			_shutdown = true;
			// We disable FETs here as well so that the user sees something happen
			_bq76->disable_fets();
			return;
		}
	}

	// Normal running operation
	app_state_s state = {};
	state.state = app_state_s::RUNNING;
	_app_state_pub.publish(state);
}

bool Bms::check_button_held()
{
	const bool button_pressed = !px4_arch_gpioread(GPIO_N_BTN); // Button press pulls to GND
	hrt_abstime now = hrt_absolute_time();

	if (button_pressed) {
		if (!_button_pressed) {
			_button_pressed = true;
			_pressed_start_time = now;
		}

		if (now - _pressed_start_time > BUTTON_HOLD_TIME) {
			_button_pressed = false;
			return true;
		}
	} else {
		// Button was previously pressed, check for how long
		if (_button_pressed) {
			hrt_abstime duration = now - _pressed_start_time;
			if (duration > 10_ms) {
				// PX4_INFO("Button pressed for %f seconds", double((float)duration/1e6f));
				_button_pressed_pub.publish(button_pressed_s{});
			}
		}

		_button_pressed = false;
	}

	return false;
}

void Bms::shutdown()
{
	_bq76->disable_fets();
	PX4_INFO("Good bye!");
	px4_usleep(50_ms);
	stm32_gpiowrite(GPIO_PWR_EN, false);
	px4_usleep(50_ms);
	ScheduleClear();
}

void Bms::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		ModuleParams::updateParams();
	}
}

int Bms::flags()
{
	uint32_t flags = _bq76->status_flags();
	PX4_INFO("flags: 0x%08lx", flags);

	if (flags & watts_battery_status_s::STATUS_FLAG_READY_TO_USE) {
		PX4_INFO("Ready to use");
	} else {
		PX4_INFO("Not ready to use");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_CHARGING) {
		PX4_INFO("Charging");
	} else {
		PX4_INFO("Discharging");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_CELL_BALANCING) {
		PX4_INFO("Balancing");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_AUTO_DISCHARGING) {
		PX4_INFO("Auto discharging");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_REQUIRES_SERVICE) {
		PX4_INFO("Requires service");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_BAD_BATTERY) {
		PX4_INFO("Bad battery");
	}

	if (flags & watts_battery_status_s::STATUS_FLAG_PROTECTIONS_ENABLED) {
		PX4_INFO("Protections enabled");
	} else {
		PX4_INFO("Protections disabled");
	}

	// Faults
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM) {
		PX4_INFO("--- SAFETY FAULT ---");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_OVER_VOLT) {
		PX4_INFO("Over volt");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_UNDER_VOLT) {
		PX4_INFO("Under volt");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_OVER_TEMP) {
		PX4_INFO("Over temp");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_UNDER_TEMP) {
		PX4_INFO("Under temp");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_OVER_CURRENT) {
		PX4_INFO("Over current");
	}
	if (flags & watts_battery_status_s::STATUS_FLAG_FAULT_SHORT_CIRCUIT) {
		PX4_INFO("Short circuit");
	}

	return PX4_OK;
}

int Bms::diagnostics()
{
	PX4_INFO("========== bq76 ==========");
	// MFG STATUS
	{
		uint16_t status = _bq76->mfg_status();
		PX4_INFO("mfg status: 0x%x", status);

		if (status & (1 << 4)) {
			PX4_INFO("Normal FET control is enabled");
		} else {
			PX4_INFO("Normal FET control is disabled");
		}

		if (status & (1 << 6)) {
			PX4_INFO("Permanent Failure checks are enabled");
		} else {
			PX4_INFO("Permanent Failure checks are disabled");
		}

		if (status & (1 << 7)) {
			PX4_INFO("Device may program OTP during operation");
		} else {
			PX4_INFO("Device will not program OTP during operation");
		}
		PX4_INFO("");
	}

	// BATTERY STATUS
	{
		uint16_t status = _bq76->battery_status();
		PX4_INFO("battery status: 0x%x", status);

		if (status & (1 << 0)) {
			PX4_INFO("Device is in CONFIG_UPDATE mode");
		} else {
			PX4_INFO("Device is not in CONFIG_UPDATE mode");
		}

		if (status & (1 << 1)) {
			PX4_INFO("Device is in PRECHARGE mode");
		} else {
			PX4_INFO("Device is not in PRECHARGE mode");
		}

		if (status & (1 << 6)) {
			PX4_INFO("Writes to OTP are pending");
		} else {
			PX4_INFO("No writes to OTP are pending");
		}

		if (status & (1 << 7)) {
			PX4_INFO("OTP writes are allowed");
		} else {
			PX4_INFO("OTP writes are blocked");
		}

		if ((status & (1 << 8)) && status & (1 << 9)) {
			PX4_INFO("Device is in SEALED mode");
		} else if (status & (1 << 8)) {
			PX4_INFO("Device is in FULLACCESS mode");
		} else if (status & (1 << 9)) {
			PX4_INFO("Device is in UNSEALED mode");
		} else {
			PX4_INFO("Device has not initialized yet");
		}

		if (status & (1 << 11)) {
			PX4_INFO("At least one enabled safety fault is triggered");
		} else {
			PX4_INFO("No safety fault is triggered");
		}

		if (status & (1 << 12)) {
			PX4_INFO("At least one Permanent Fail fault has triggered.");
		} else {
			PX4_INFO("No Permanent Fail fault has triggered.");
		}
		PX4_INFO("");
	}

	// OTP_WR_CHECK
	{
		uint8_t status = _bq76->otp_wr_check();

		PX4_INFO("otp_wr_check: 0x%x", status);

		if (status & (1 << 7)) {
			PX4_INFO("OTP writes enabled");
		} else {
			PX4_INFO("OTP writes disabled");
		}

		if (status & (1 << 5)) {
			PX4_INFO("The device is not in FULLACCESS and CONFIG_UPDATE mode, or the OTP Lock bit has been set to prevent further modification");
		}

		if (status & (1 << 4)) {
			PX4_INFO("Signature cannot be written (indicating the signature has already been written too many times)");
		}

		if (status & (1 << 3)) {
			PX4_INFO("Could not program data (indicating data has been programmed too many times; no XOR bits left)");
		}

		if (status & (1 << 2)) {
			PX4_INFO("The measured internal temperature is above the allowed OTP programming temperature range");
		}

		if (status & (1 << 1)) {
			PX4_INFO("The measured stack voltage is below the allowed OTP programming voltage");
		}

		if (status & (1 << 0)) {
			PX4_INFO("The measured stack voltage is above the allowed OTP programming voltage");
		}
	}

	PX4_INFO("\n========== bq34 ==========");
	// Control status
	{
		uint16_t status = _bq34->read_control_status();
		PX4_INFO("control status: 0x%x", status);

		if (status & (1 << 14)) {
			PX4_INFO("bq34z100-G1 is in FULL ACCESS SEALED state");
		}

		if (status & (1 << 13)) {
			PX4_INFO("bq34z100-G1 is in the SEALED State");
		}

		if (status & (1 << 12)) {
			PX4_INFO("calibration function is active");
		}

		if (status & (1 << 11)) {
			PX4_INFO("Coulomb Counter Calibration routine is active.");
		}

		if (status & (1 << 10)) {
			PX4_INFO("Board Calibration routine is active");
		}

		if (status & (1 << 9)) {
			PX4_INFO("a valid data flash checksum has been generated");
		}

		if (status & (1 << 4)) {
			PX4_INFO("bq34z100-G1 is in SLEEP mode");
		}

		if (status & (1 << 3)) {
			PX4_INFO("Impedance Track algorithm using constant-power mode");
		}

		if (status & (1 << 2)) {
			PX4_INFO("Ra table updates are disabled");
		}

		if (status & (1 << 1)) {
			PX4_INFO("cell voltages are OK for Qmax updates");
		}

		if (status & (1 << 0)) {
			PX4_INFO("Qmax updates are enabled");
		}
	}

	// Device type
	{
		uint16_t type = _bq34->read_device_type();
		PX4_INFO("device type: 0x%x", type);
	}

	return PX4_OK;
}

int Bms::on()
{
	_bq76->enable_fets();
	return PX4_OK;
}

int Bms::off()
{
	_bq76->disable_fets();
	return PX4_OK;
}

int Bms::disable_protections()
{
	_bq76->disable_protections();
	return PX4_OK;
}

int Bms::enable_protections()
{
	_bq76->enable_protections();
	return PX4_OK;
}

int Bms::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "diag")) {
		if (is_running()) {
			return _object.load()->diagnostics();
		}
		return PX4_ERROR;
	}

	if (!strcmp(verb, "flags")) {
		if (is_running()) {
			return _object.load()->flags();
		}
		return PX4_ERROR;
	}

	if (!strcmp(verb, "on")) {
		if (is_running()) {
			return _object.load()->on();
		}
		return PX4_ERROR;
	}

	if (!strcmp(verb, "off")) {
		if (is_running()) {
			return _object.load()->off();
		}
		return PX4_ERROR;
	}

	if (!strcmp(verb, "disable_p")) {
		if (is_running()) {
			return _object.load()->disable_protections();
		}
		return PX4_ERROR;
	}

	if (!strcmp(verb, "enable_p")) {
		if (is_running()) {
			return _object.load()->enable_protections();
		}
		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int Bms::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
BMS driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bms", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("diag", "Prints a bunch of helpful diagnostic info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("flags", "Prints bq76 safety fault flags");
	PRINT_MODULE_USAGE_COMMAND_DESCR("on", "Enables the output FETs");
	PRINT_MODULE_USAGE_COMMAND_DESCR("off", "Disables the output FETs");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable_p", "Disables protections");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable_p", "Enables protections");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int Bms::task_spawn(int argc, char *argv[])
{
	Bms *instance = new Bms();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int bms_main(int argc, char *argv[])
{
	return Bms::main(argc, argv);
}
