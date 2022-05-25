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

// FUNCTION OF THE BQ76952
// Enable bq34
// Read temperature
// Read cell voltages [12]
// Read current
// Read fault (Battery Status)

// FUNCTION OF THE BQ34Z100
// Read current consumed
// Read energy consumed
// Read percent remaining
// Read time remaining
// Read mAh consumed

#include "Bms.hpp"

Bms::Bms() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::I2C1),
	_cycle_perf(perf_alloc(PC_ELAPSED, "Bms: single-sample"))
{
	_battery_status_pub.advertise();
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

	if (_bq76->configure_settings() != PX4_OK) {
		return PX4_ERROR;
	}

	// Set FET mode to normal and configure FET actions when protections are tripped
	_bq76->configure_fets();

	// should we let the auto protection enable do this?
	_bq76->enable_protections();

	// TODO: just a test right now
	_bq76->read_manu_data();

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

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

	// Detect button presses and handle corresponding behavior
	handle_button_and_boot();

	// If drawing a very low amount of power for some amount of time, automatically turn pack off
	handle_idle_current_detection();

	// Automatically enable/disable protections
	handle_automatic_protections();

	collect_and_publish();

	perf_end(_cycle_perf);
}

void Bms::collect_and_publish()
{
	watts_battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	int ret = PX4_OK;

	// Read external thermistor on TS3
	// TODO: sporadic erroneous values come from the BQ34 having TS enabled in the TEMPS bit mask in the Pack Configuration Register
	int16_t temperature = {};
	ret |= _bq76->direct_command(CMD_READ_TS3_TEMP, &temperature, sizeof(temperature));
	px4_usleep(1_ms);
	battery_status.temperature = ((float)temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C

	int16_t current = {};
	ret |= _bq76->direct_command(CMD_READ_CC2_CURRENT, &current, sizeof(current));
	px4_usleep(1_ms);
	battery_status.current = (float)current / 100.0f; // centi-amp resolution 327amps +/-

	int16_t stack_voltage = {};
	ret |= _bq76->direct_command(CMD_READ_STACK_VOLTAGE, &stack_voltage, sizeof(stack_voltage));
	px4_usleep(1_ms);
	battery_status.voltage = (float)stack_voltage / 100.0f; // centi-volt

	// ignore capacity consumed
	// Read capacity remaining
	battery_status.capacity_remaining = _bq34->read_remaining_capacity();

	// Read cell voltages
	int16_t cell_voltages_mv[12] = {};
	ret |= _bq76->direct_command(CMD_READ_CELL_VOLTAGE, &cell_voltages_mv, sizeof(cell_voltages_mv));
	px4_usleep(1_ms);

	for (size_t i = 0; i < sizeof(cell_voltages_mv) / sizeof(cell_voltages_mv[0]); i++) {
		battery_status.cell_voltages[i] = cell_voltages_mv[i] / 1000.0f;
	}

	battery_status.design_capacity = _bq34->read_design_capacity();
	battery_status.actual_capacity = _bq34->read_full_charge_capacity();
	battery_status.cycle_count = _bq34->read_cycle_count();
	battery_status.state_of_health = _bq34->read_state_of_health();

	battery_status.status_flags = _bq76->get_status_flags();

	_battery_status_pub.publish(battery_status);
}

void Bms::handle_automatic_protections()
{
	bool auto_protect = _param_auto_protect.get();

	if (!auto_protect) {
		return;
	}

	int16_t data = {};
	int ret = _bq76->direct_command(CMD_READ_CC2_CURRENT, &data, sizeof(data));

	if (ret != PX4_OK) {
		return;
	}

	float current = (float)data / 100.0f;
	float protect_current = _param_protect_current.get();

	// TODO: current > threshold for X seconds

	if (current > protect_current) {
		if (_protections_enabled) {
			PX4_INFO("TODO: Current exceeds PROTECT_CURRENT (%2.2f), disabling protections", double(protect_current));
			// disable_protections();
			_protections_enabled = false;
		}
	} else {
		if (!_protections_enabled) {
			PX4_INFO("TODO: Current is below PROTECT_CURRENT (%2.2f), enabling protections", double(protect_current));
			// enable_protections();
			_protections_enabled = true;
		}
	}
}

void Bms::handle_idle_current_detection()
{
	int32_t idle_timeout = _param_idle_timeout.get();
	if (idle_timeout == 0) {
		return;
	}

	int16_t data = {};
	int ret = _bq76->direct_command(CMD_READ_CC2_CURRENT, &data, sizeof(data));
	float current = (float)data / 100.0f;

	if (ret != PX4_OK) {
		return;
	}

	hrt_abstime now = hrt_absolute_time();
	if (current < _param_idle_current.get()) {
		if (!_below_idle_current) {
			_below_idle_current = true;
			_idle_start_time = now;
		}

		hrt_abstime timeout = (hrt_abstime)idle_timeout * 1e6;
		if (now > _idle_start_time + timeout) {
			PX4_INFO("Battery has been idle for %llu, shutting down", timeout);
			_shutdown_pub.publish(shutdown_s{});
			_shutdown = true;
		}

	} else {
		_below_idle_current = false;
	}
}

void Bms::handle_button_and_boot()
{
	if (!_booted) {
		if (check_button_held()) {
			PX4_INFO("Button was held, enabling FETs");
			_booted = true;
			_booted_button_held = true;
			_bq76->enable_fets();
		}

		// Check if PACK voltage is high
		int16_t pack_voltage = {};
		_bq76->direct_command(CMD_READ_PACK_PIN_VOLTAGE, &pack_voltage, sizeof(pack_voltage));
		px4_usleep(1_ms);
		float pack_voltage_f = pack_voltage / 100.0f;
		float voltage_threshold = _param_parallel_voltage.get();
		if (pack_voltage_f >= voltage_threshold) {
			PX4_INFO("PACK voltage (%fv) above threshold (%fv), enabling FETs", double(pack_voltage_f), double(voltage_threshold));
			_booted = true;
			_bq76->enable_fets();
			return;
		}

		// Check if 5 seconds has elapsed, power off
		if (hrt_absolute_time() > 5_s) {
			PX4_INFO("Button not held, shutting down");
			_shutdown_pub.publish(shutdown_s{});
			_shutdown = true;
		}

	} else if (_booted && _booted_button_held) {
		// We must make sure the button is released before we start monitoring for shutdown / screen toggle
		if (px4_arch_gpioread(GPIO_N_BTN)) {
			PX4_INFO("button released after boot");
			_booted_button_held = false;
		}

	} else {
		if (check_button_held()) {
			PX4_INFO("Button was held, shutting down");
			// Notify shutdown
			_shutdown_pub.publish(shutdown_s{});
			_shutdown = true;
		}
	}
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

		static constexpr hrt_abstime HOLD_TIME = 3_s;
		if (now - _pressed_start_time > HOLD_TIME) {
			_button_pressed = false;
			return true;
		}
	} else {
		// Button was previously pressed, check for how long
		if (_button_pressed) {
			hrt_abstime duration = now - _pressed_start_time;
			if (duration > 10_ms) {
				PX4_INFO("Button pressed for %f seconds", double((float)duration/1e6f));
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
}

void Bms::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		// update parameters from storage
		ModuleParams::updateParams();
	}
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
	// Enable LDO at REG1 if it's not already enabled (turns on the bq34)
	uint8_t value = 0b00001101;
	_bq76->sub_command(CMD_REG12_CONTROL, &value, sizeof(value));
	px4_usleep(5_ms);

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

int Bms::otp_check()
{
	_bq76->enter_config_update_mode();
	_bq76->sub_command(CMD_OTP_WR_CHECK);
	px4_usleep(5_ms);
	uint16_t status = _bq76->sub_command_response8(0);

	if (status & (1 << 7)) {
		PX4_INFO("OTP writes enabled: %x", status);
	} else {
		PX4_INFO("OTP writes disabled: %x", status);
	}

	uint8_t buf[2] = {};
	_bq76->direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	uint16_t battery_status = (buf[1] << 8) + buf[0];
	PX4_INFO("battery status: 0x%x", battery_status);
	// 0000 1001 1000 0101
	if (battery_status & (1 << 7)) {
		PX4_INFO("OTP writes blocked due to voltage/temperature");
	}

	_bq76->exit_config_update_mode();
	return PX4_OK;
}

int Bms::read_manu()
{
	_bq76->read_manu_data();
	return PX4_OK;
}

int Bms::write_manu()
{
	PX4_INFO("Trying to write MANU_DATA");
	_bq76->enter_config_update_mode();
	_bq76->sub_command(CMD_OTP_WR_CHECK);
	px4_usleep(5_ms);
	uint16_t status = _bq76->sub_command_response8(0);
	if (status & (1 << 7)) {
		const char* str = "this is a test";
		PX4_INFO("Writing %s", str);
		_bq76->sub_command(CMD_MANU_DATA, (void*)str, sizeof(str));
		px4_usleep(50_ms);
	} else {
		PX4_INFO("OTP writes disabled");
	}

	_bq76->exit_config_update_mode();
	return PX4_OK;
}

int Bms::mfg()
{
	_bq76->sub_command(CMD_MFG_STATUS);
	px4_usleep(5_ms);
	uint16_t status = _bq76->sub_command_response16(0);
	PX4_INFO("mfg status: 0x%x", status);
	return PX4_OK;
}

int Bms::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "otp_check")) {
		if (is_running()) {
			return _object.load()->otp_check();
		}

		return PX4_ERROR;
	}

	if (!strcmp(verb, "read_manu")) {
		if (is_running()) {
			return _object.load()->read_manu();
		}

		return PX4_ERROR;
	}

	if (!strcmp(verb, "write_manu")) {
		if (is_running()) {
			return _object.load()->write_manu();
		}

		return PX4_ERROR;
	}

	if (!strcmp(verb, "mfg")) {
		if (is_running()) {
			return _object.load()->mfg();
		}

		return PX4_ERROR;
	}

	return print_usage("unknown command");
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
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bms_main(int argc, char *argv[])
{
	return Bms::main(argc, argv);
}
