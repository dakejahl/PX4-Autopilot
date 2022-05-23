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

#include "BQ76952.hpp"

BQ76952::BQ76952(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
	_battery_status_pub.advertise();
}

BQ76952::~BQ76952()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
	_battery_status_pub.unadvertise();
	if (_bq34) delete _bq34;
}

void BQ76952::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

int BQ76952::init()
{
	PX4_INFO("Initializing BQ76952");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	update_params(true);

	///// WRITE SETTINGS INTO PERMANENT MEMORY /////

	// TODO: Set TS3 config for use with 10k external thermistor
	// xxxxxx
	// 00: 18k pull-up
	// 00: 18k temp model
	// 01: thermistor temperature measurement, used for cell temperature protections
	// xx
	// 11: ADC Input or Thermistor
	uint8_t ts3_config = 0b00000111;
	ret = write_memory8(ADDR_TS3_CONFIG, ts3_config);

	// Set current and voltage resolution (DA Configuration: USER_AMPS_0 and USER_AMPS_1)
	ret = write_memory8(ADDR_DA_CONFIG, DA_CONFIG_CENTIVOLT_CENTIAMP);
	if (ret != PX4_OK) {
		PX4_ERR("writing DA_CONFIG failed");
		return PX4_ERROR;
	}

	// Set REG1 voltage to 3.3v
	ret = write_memory8(ADDR_REG12_CONFIG, REG1_ENABLE_3v3);
	if (ret != PX4_OK) {
		PX4_ERR("writing REG12 failed");
		return PX4_ERROR;
	}

	// Enable regulator(s)
	ret = write_memory8(ADDR_REG0, 0x01); // Enable the bq32z100
	if (ret != PX4_OK) {
		PX4_ERR("writing REG0 failed");
		return PX4_ERROR;
	}

	// Enable LDO at REG1 if it's not already enabled
	uint8_t value = 0b00001101;
	sub_command(CMD_REG12_CONTROL, &value, sizeof(value));
	px4_usleep(5_ms);

	// TODO: cell low voltage cutoff in-air and on-ground

	configure_protections();
	disable_protections();

	read_manu_data();

	// Enable the BQ34
	_bq34 = new BQ34Z100();

	if (!_bq34) {
		PX4_INFO("failed to create BQ34Z100");
		return PX4_ERROR;
	}

	if (_bq34->init() != PX4_OK) {
		PX4_INFO("failed to initialize BQ34Z100");
		return PX4_ERROR;
	}

	// Check Manufacturing Status register
	{
		// TODO: Alex's is at 0x10
		sub_command(CMD_MFG_STATUS);
		px4_usleep(5_ms);
		uint16_t status = sub_command_response16(0);
		PX4_INFO("mfg status: 0x%x", status);

		// Set FET "normal mode" if not already set
		if (!(status & (1 << 4))) {
			PX4_INFO("Enabling FET normal mode");
			sub_command(CMD_FET_ENABLE);
		}
	}

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

void BQ76952::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("exiting");
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

void BQ76952::collect_and_publish()
{
	watts_battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	int ret = PX4_OK;

	// Read external thermistor on TS3 -- TODO: values are wrong
	int16_t temperature = {};
	ret |= direct_command(CMD_READ_TS3_TEMP, &temperature, sizeof(temperature));
	// raw value
	battery_status.temperature = temperature; // Convert from 0.1K to C
	// battery_status.temperature = ((float)temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C

	int16_t current = {};
	ret |= direct_command(CMD_READ_CC2_CURRENT, &current, sizeof(current));
	px4_usleep(1_ms);
	battery_status.current = (float)current / 100.0f; // centi-amp resolution 327amps +/-

	int16_t stack_voltage = {};
	ret |= direct_command(CMD_READ_STACK_VOLTAGE, &stack_voltage, sizeof(stack_voltage));
	px4_usleep(1_ms);
	battery_status.voltage = (float)stack_voltage / 100.0f;

	// ignore capacity consumed
	// Read capacity remaining
	battery_status.capacity_remaining = _bq34->read_remaining_capacity();

	// Read cell voltages
	int16_t cell_voltages_mv[12] = {};
	ret |= direct_command(CMD_READ_CELL_VOLTAGE, &cell_voltages_mv, sizeof(cell_voltages_mv));
	px4_usleep(1_ms);

	for (size_t i = 0; i < sizeof(cell_voltages_mv) / sizeof(cell_voltages_mv[0]); i++) {
		battery_status.cell_voltages[i] = cell_voltages_mv[i] / 1000.0f;
	}

	battery_status.design_capacity = _bq34->read_design_capacity();
	battery_status.actual_capacity = _bq34->read_full_charge_capacity();
	battery_status.cycle_count = _bq34->read_cycle_count();
	battery_status.state_of_health = _bq34->read_state_of_health();

	battery_status.status_flags = get_status_flags();

	_battery_status_pub.publish(battery_status);
}

uint32_t BQ76952::get_status_flags()
{
	uint32_t status_flags = {};

	// static constexpr uint32_t STATUS_FLAG_IN_USE               = 1;
	// static constexpr uint32_t STATUS_FLAG_READY_TO_USE         = 2;
	// static constexpr uint32_t STATUS_FLAG_CHARGING             = 4;
	static constexpr uint32_t STATUS_FLAG_OVER_TEMP            = 8;
	static constexpr uint32_t STATUS_FLAG_UNDER_TEMP           = 16;
	static constexpr uint32_t STATUS_FLAG_OVER_VOLT            = 32;
	static constexpr uint32_t STATUS_FLAG_UNDER_VOLT           = 64;
	static constexpr uint32_t STATUS_FLAG_OVER_CURRENT         = 128;
	static constexpr uint32_t STATUS_FLAG_SHORT_CIRCUIT        = 256;
	static constexpr uint32_t STATUS_FLAG_SAFETY_FAULT         = 512;
	// static constexpr uint32_t STATUS_FLAG_CELL_IMBALANCE       = 1024;
	// static constexpr uint32_t STATUS_FLAG_CELL_BALANCING       = 2048;
	// static constexpr uint32_t STATUS_FLAG_CELL_FAULT           = 4096;
	// static constexpr uint32_t STATUS_FLAG_PROTECTIONS_ENABLED  = 8192;
	static constexpr uint32_t STATUS_FLAG_REQUIRES_SERVICE     = 16384;

	// SAFETY ALERT/STATUS A
	{
		int ret = PX4_OK;
		uint8_t safety_alert_a = {};
		uint8_t safety_status_a = {};
		ret |= direct_command(CMD_SAFETY_ALERT_A, &safety_alert_a, sizeof(safety_alert_a));
		ret |= direct_command(CMD_SAFETY_STATUS_A, &safety_status_a, sizeof(safety_status_a));

		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t under_volt_mask = (1 << 2);
		if (safety_alert_a & under_volt_mask) {
			status_flags |= STATUS_FLAG_UNDER_VOLT;
			if (safety_status_a & under_volt_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t over_volt_mask = (1 << 3);
		if (safety_alert_a & over_volt_mask) {
			status_flags |= STATUS_FLAG_OVER_VOLT;
			if (safety_status_a & over_volt_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t over_current_mask = (1 << 4) | (1 << 5) | (1 << 6);
		if (safety_alert_a & over_current_mask) {
			status_flags |= STATUS_FLAG_OVER_CURRENT;
			if (safety_status_a & over_current_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t short_circuit_mask = (1 << 7);
		if (safety_alert_a & short_circuit_mask) {
			status_flags |= STATUS_FLAG_SHORT_CIRCUIT;
			if (safety_status_a & short_circuit_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}
	}

	// SAFETY ALERT/STATUS B
	{
		int ret = PX4_OK;
		uint8_t safety_alert_b = {};
		uint8_t safety_status_b = {};
		ret |= direct_command(CMD_SAFETY_ALERT_B, &safety_alert_b, sizeof(safety_alert_b));
		ret |= direct_command(CMD_SAFETY_STATUS_B, &safety_status_b, sizeof(safety_status_b));

		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_temp_mask = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4);
		if (safety_alert_b & over_temp_mask) {
			status_flags |= STATUS_FLAG_OVER_TEMP;
			if (safety_status_b & over_temp_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t under_temp_mask = (1 << 0) | (1 << 1) | (1 << 2);
		if (safety_alert_b & under_temp_mask) {
			status_flags |= STATUS_FLAG_UNDER_TEMP;
			if (safety_status_b & under_temp_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}
	}

	// SAFETY ALERT/STATUS C
	{
		int ret = PX4_OK;
		uint8_t safety_alert_c = {};
		uint8_t safety_status_c = {};
		ret |= direct_command(CMD_SAFETY_ALERT_C, &safety_alert_c, sizeof(safety_alert_c));
		ret |= direct_command(CMD_SAFETY_STATUS_C, &safety_status_c, sizeof(safety_status_c));

		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_current_mask = (1 << 7) | (1 << 5);
		if (safety_alert_c & over_current_mask) {
			status_flags |= STATUS_FLAG_OVER_CURRENT;
			if (safety_status_c & over_current_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t short_circuit_mask = (1 << 6);
		if (safety_alert_c & short_circuit_mask) {
			status_flags |= STATUS_FLAG_SHORT_CIRCUIT;
			if (safety_status_c & short_circuit_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}

		uint8_t over_volt_mask = (1 << 4);
		if (safety_alert_c & over_volt_mask) {
			status_flags |= STATUS_FLAG_OVER_VOLT;
			if (safety_status_c & over_volt_mask) {
				status_flags |= STATUS_FLAG_SAFETY_FAULT;
			}
		}
	}

	return status_flags;
}

void BQ76952::handle_automatic_protections()
{
	bool auto_protect = _param_auto_protect.get();

	if (!auto_protect) {
		return;
	}

	int16_t data = {};
	int ret = direct_command(CMD_READ_CC2_CURRENT, &data, sizeof(data));

	if (ret != PX4_OK) {
		return;
	}

	float current = (float)data / 100.0f;
	float protect_current = _param_protect_current.get();

	if (current > protect_current) {
		if (_protections_enabled) {
			// Disable protections
			PX4_INFO("TODO: Current exceeds PROTECT_CURRENT (%2.2f), disabling protections", double(protect_current));
			_protections_enabled = false;
		}
	} else {
		if (!_protections_enabled) {
			// Enable protections
			PX4_INFO("TODO: Current is below PROTECT_CURRENT (%2.2f), enabling protections", double(protect_current));
			_protections_enabled = true;
		}
	}
}

void BQ76952::handle_idle_current_detection()
{
	int32_t idle_timeout = _param_idle_timeout.get();
	if (idle_timeout == 0) {
		return;
	}

	int16_t data = {};
	int ret = direct_command(CMD_READ_CC2_CURRENT, &data, sizeof(data));
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

void BQ76952::handle_button_and_boot()
{
	if (!_booted) {
		if (check_button_held()) {
			PX4_INFO("Button was held, enabling FETs");
			_booted = true;
			_booted_button_held = true;
			enable_fets();
		}

		// Check if PACK voltage is high
		int16_t pack_voltage = {};
		direct_command(CMD_READ_PACK_PIN_VOLTAGE, &pack_voltage, sizeof(pack_voltage));
		px4_usleep(1_ms);
		float pack_voltage_f = pack_voltage / 100.0f;
		float voltage_threshold = _param_parallel_voltage.get();
		if (pack_voltage_f >= voltage_threshold) {
			PX4_INFO("PACK voltage (%fv) above threshold (%fv), booting", double(pack_voltage_f), double(voltage_threshold));
			_booted = true;
			return;
		}

		// Check if 5 seconds has elapsed, power off
		if (hrt_absolute_time() > 5_s) {
			PX4_INFO("Button not held");
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
			PX4_INFO("Button was held, disabling FETs");
			disable_fets();
			// Notify shutdown
			_shutdown_pub.publish(shutdown_s{});
			_shutdown = true;
		}
	}
}

bool BQ76952::check_button_held()
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

void BQ76952::shutdown()
{
	disable_fets();
	PX4_INFO("Good bye!");
	px4_usleep(50_ms);
	stm32_gpiowrite(GPIO_PWR_EN, false);
	px4_usleep(50_ms);
}

void BQ76952::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		// update parameters from storage
		ModuleParams::updateParams();
	}
}

void BQ76952::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bq76952", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void BQ76952::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

int BQ76952::probe()
{
	uint8_t val = {};
	int ret = transfer(&val, sizeof(val), nullptr, 0);
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void BQ76952::configure_protections()
{
	// ADDR_PROTECTION_CONFIG
	// SCDL_CURR_RECOV -- 1 = SCDL recovers when current is greater than or equal to Protections:SCDL:RecoveryTime.
	// OCDL_CURR_RECOV -- 1 = OCDL recovers when current is greater than or equal to Protections:OCDL:RecoveryTime
	// PF_OTP -- If this bit is not set, Permanent Failure status will be lost on any reset

	//  The individual protections can be enabled by setting the related Settings:Protection:Enabled Protections
	//  A – C configuration registers.

	// Settings:Protection:Enabled Protections A
	{
		uint8_t byte = {};

		// 7 SCD 1 Short Circuit in Discharge Protection
		byte |= 1 << 7;

		// 6 OCD2 0 Overcurrent in Discharge 2nd Tier Protection
		// byte |= 1 << 6;

		// 5 OCD1 0 Overcurrent in Discharge 1st Tier Protection
		// byte |= 1 << 5;

		// 4 OCC 0 Overcurrent in Charge Protection
		// byte |= 1 << 4;

		// 3 COV 1 Cell Overvoltage Protection
		byte |= 1 << 3;

		// 2 CUV 0 Cell Undervoltage Protection
		// byte |= 1 << 2;

		write_memory8(ADDR_PROTECTIONS_A, byte);
	}

	// Settings:Protection:Enabled Protections B
	{
		uint8_t byte = {};

		// 7 OTF 0 FET Overtemperature
		// byte |= 1 << 7;

		// 6 OTINT 0 Internal Overtemperature
		// byte |= 1 << 6;

		// 5 OTD 0 Overtemperature in Discharge
		// byte |= 1 << 5;

		// 4 OTC 0 Overtemperature in Charge
		// byte |= 1 << 4;

		// 2 UTINT 0 Internal Undertemperature
		// byte |= 1 << 2;

		// 1 UTD 0 Undertemperature in Discharge
		// byte |= 1 << 1;

		// 0 UTC 0 Undertemperature in Charge
		// byte |= 1 << 0;

		write_memory8(ADDR_PROTECTIONS_B, byte);
	}

	// Settings:Protection:Enabled Protections C
	{
		uint8_t byte = {};

		// 7 OCD3 0 Overcurrent in Discharge 3rd Tier Protection
		// byte |= 1 << 7;

		// 6 SCDL 0 Short Circuit in Discharge Latch
		// byte |= 1 << 6;

		// 5 OCDL 0 Overcurrent in Discharge Latch
		// byte |= 1 << 5;

		// 4 COVL 0 Cell Overvoltage Latch
		// byte |= 1 << 4;

		// 2 PTO 0 Precharge Timeout
		// byte |= 1 << 2;

		// 1 HWDF 0 Host Watchdog Fault
		// byte |= 1 << 1;

		write_memory8(ADDR_PROTECTIONS_C, byte);
	}

	// Settings:Manufacturing:Mfg Status Init[FET_EN] -- autonomous control mode?
}

void BQ76952::enable_protections()
{
	// CHG FET Protections A
	{
		uint8_t byte = {};

		// 7 SCD 1 Short Circuit in Discharge Protection
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 7);

		// 4 OCC 1 Overcurrent in Charge Protection
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);

		// 3 COV 1 Cell Overvoltage Protection
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 3);

		write_memory8(ADDR_CHG_FET_Protections_A, byte);
	}

	// CHG FET Protections B
	{
		uint8_t byte = {};

		// 7 OTF 1 FET Overtemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 7);

		// 6 OTINT 1 Internal Overtemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 6);

		// 4 OTC 1 Overtemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);

		// 2 UTINT 1 Internal Undertemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 2);

		// 0 UTC 1 Undertemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);

		write_memory8(ADDR_CHG_FET_Protections_B, byte);
	}

	// CHG FET Protections C
	{
		uint8_t byte = {};

		// 6 SCDL 1 Short Circuit in Discharge Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (6 << 0);

		// 4 COVL 1 Cell Overvoltage Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (4 << 0);

		// 2 PTO 1 Precharge Timeout
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (2 << 0);

		// 1 HWDF 1 Host Watchdog Fault
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);

		write_memory8(ADDR_CHG_FET_Protections_C, byte);
	}
}

void BQ76952::read_manu_data()
{
	PX4_INFO("read_manu_data");
	// Read MANU_DATA
	uint8_t manu_data[32] = {};
	px4_usleep(5_ms);
	sub_command(CMD_MANU_DATA);
	// px4_usleep(500_ms);
	sub_command_response_buffer(manu_data, sizeof(manu_data));

	for (size_t i = 0; i < sizeof(manu_data); i++) {
		printf("%c ", manu_data[i]);
	}
	printf("\n");
}

void BQ76952::disable_protections()
{
	uint8_t byte = {};
	write_memory8(ADDR_CHG_FET_Protections_A, byte);
	write_memory8(ADDR_CHG_FET_Protections_B, byte);
	write_memory8(ADDR_CHG_FET_Protections_C, byte);
	write_memory8(ADDR_DSG_FET_Protections_A, byte);
	write_memory8(ADDR_DSG_FET_Protections_B, byte);
	write_memory8(ADDR_DSG_FET_Protections_C, byte);
}

void BQ76952::enable_fets()
{
	sub_command(CMD_ALL_FETS_ON);
	px4_usleep(5_ms);
}

void BQ76952::disable_fets()
{
	sub_command(CMD_ALL_FETS_OFF);
	px4_usleep(5_ms);
}

uint8_t BQ76952::read_memory8(uint16_t addr)
{
	uint8_t buf[3] = {};
	buf[0] = CMD_ADDR_SUBCMD_LOW;
	buf[1] = addr & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	uint8_t data = 0;
	int ret = transfer(buf, sizeof(buf), &data, sizeof(data));
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return data;
}

uint16_t BQ76952::read_memory16(uint16_t addr)
{
	uint8_t buf[3] = {};
	buf[0] = CMD_ADDR_SUBCMD_LOW;
	buf[1] = addr & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	uint16_t data = 0;
	// int ret = transfer(buf, sizeof(buf), (uint8_t*)&data, sizeof(data));
	int ret = transfer(buf, sizeof(buf), nullptr, 0);
	px4_usleep(5_ms);
	ret = transfer(nullptr, 0, (uint8_t*)&data, sizeof(data));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return data;
}

int BQ76952::write_memory8(uint16_t addr, uint8_t data)
{
	PX4_INFO("Writing to 0x%x --> 0x%x", addr, data);

	// Must be in config update mode to write to memory
	int ret = enter_config_update_mode();
	if (ret != PX4_OK) {
		PX4_ERR("failed to enter config update mode");
		return PX4_ERROR;
	}

	// See pg 13 of technical reference
	// The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
	// number of bytes used in the transfer buffer, then the result is bitwise inverted
	uint8_t checksum = 0;
	// Send the data
	{
		uint8_t buf[4] = {};
		buf[0] = CMD_ADDR_SUBCMD_LOW;
		buf[1] = uint8_t(addr & 0x00FF);
		buf[2] = uint8_t((addr >> 8) & 0x00FF);
		buf[3] = data;

		ret |= transfer(buf, sizeof(buf), nullptr, 0);
		for (size_t i = 1; i < sizeof(buf); i++) {
			checksum += buf[i];
		}
	}

	// Send checksum and length
	{
		uint8_t buf[3] = {};
		buf[0] = CMD_ADDR_RESP_CHKSUM;
		buf[1] = ~checksum;
		buf[2] = 5; // 2 bytes addr, 1 bytes data, 1 byte checksum, 1 byte length
		ret |= transfer(buf, sizeof(buf), nullptr, 0);
	}

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	exit_config_update_mode();

	return PX4_OK;
}

int BQ76952::write_memory16(uint16_t addr, uint16_t data)
{
	PX4_INFO("Writing to 0x%x --> 0x%x", addr, data);

	int ret = enter_config_update_mode();
	if (ret != PX4_OK) {
		PX4_ERR("failed to write memory");
		return PX4_ERROR;
	}

	// See pg 13 of technical reference
	// The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
	// number of bytes used in the transfer buffer, then the result is bitwise inverted
	uint8_t checksum = 0;
	// Send the data
	{
		uint8_t buf[5] = {};
		buf[0] = CMD_ADDR_SUBCMD_LOW;
		buf[1] = uint8_t(addr & 0x00FF);
		buf[2] = uint8_t((addr >> 8) & 0x00FF);
		buf[3] = uint8_t(data & 0x00FF);
		buf[4] = uint8_t((data >> 8) & 0x00FF);

		ret |= transfer(buf, sizeof(buf), nullptr, 0);
		for (size_t i = 1; i < sizeof(buf); i++) {
			checksum += buf[i];
		}
	}

	// Send checksum and length
	{
		uint8_t buf[3] = {};
		buf[0] = CMD_ADDR_RESP_CHKSUM;
		buf[1] = ~checksum;
		buf[2] = 6; // 2 bytes addr, 2 bytes data, 1 byte checksum, 1 byte length
		ret |= transfer(buf, sizeof(buf), nullptr, 0);
	}

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	exit_config_update_mode();

	return PX4_OK;
}

int BQ76952::enter_config_update_mode()
{
	// Enter config udpate mode if not already in it and report status
	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	px4_usleep(1_ms);
	if (!(buf[0] & 0x01)) {
		sub_command(CMD_SET_CFGUPDATE);
		px4_usleep(5_ms); // 2000us time to complete operation
		direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
		px4_usleep(5_ms);
		if (buf[0] & 0x01) {
			return PX4_OK;
		}
	} else {
		// Already in config update mode
		return PX4_OK;
	}

	return PX4_ERROR;
}

int BQ76952::exit_config_update_mode()
{
	sub_command(CMD_EXIT_CFG_UPDATE);
	px4_usleep(2_ms); // 1000us time to complete operation

	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	px4_usleep(5_ms);
	if (!(buf[0] & 0x01)) {
		return PX4_OK;
	}
	return PX4_ERROR;
}

int BQ76952::direct_command(uint8_t command, void* rx_buf, size_t rx_len)
{
	int ret = transfer(&command, 1, (uint8_t*)rx_buf, rx_len);
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}
	return ret;
}

int BQ76952::sub_command(uint16_t command, void* tx_buf, size_t tx_len)
{
	// Send the sub command
	uint8_t buf[3] = {};
	buf[0] = CMD_ADDR_SUBCMD_LOW;
	buf[1] = uint8_t(command & 0x00FF);
	buf[2] = uint8_t((command >> 8) & 0x00FF);
	int ret = transfer(buf, sizeof(buf), nullptr, 0);

	// Write data to the transfer buffer
	if (tx_buf != nullptr && tx_len != 0) {
		uint8_t data_buf[tx_len + 1] = {}; // data + checksum + length
		data_buf[0] = CMD_ADDR_TRANSFER_BUFFER;
		memcpy(data_buf + 1, (uint8_t*)tx_buf, tx_len);
		ret |= transfer(data_buf, sizeof(data_buf), nullptr, 0);

		// The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
		// number of bytes used in the transfer buffer, then the result is bitwise inverted.
		uint8_t crc_buf[3] = {}; // data + checksum + length
		uint8_t checksum = buf[1] + buf[2];
		for (size_t i = 0; i < tx_len; i++) {
			checksum += data_buf[i];
		}
		crc_buf[0] = CMD_ADDR_RESP_CHKSUM;
		crc_buf[1] = ~checksum;
		crc_buf[2] = 2 + tx_len + 2; // length: 2 addr bytes, data len, 1 crc, 1 length

		ret |= transfer(crc_buf, sizeof(crc_buf), nullptr, 0);
	}

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}
	return ret;
}

uint8_t BQ76952::sub_command_response8(uint8_t offset)
{
	uint8_t addr = CMD_ADDR_TRANSFER_BUFFER + offset;
	uint8_t resp = {};

	int ret = transfer(&addr, 1, &resp, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return resp;
}

uint16_t BQ76952::sub_command_response16(uint8_t offset)
{
	uint8_t addr = CMD_ADDR_TRANSFER_BUFFER + offset;
	uint8_t buf[2] = {};

	int ret = transfer(&addr, 1, buf, sizeof(buf));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return (buf[1] << 8) | buf[0];

	// return (buf[0] << 8) | buf[1];
}

int BQ76952::sub_command_response_buffer(uint8_t* buf, size_t length)
{
	// TODO: handle timeout
	// uint16_t command = CMD_MANU_DATA;
	// uint8_t low_byte = uint8_t(command & 0x00FF);
	// uint8_t high_byte = uint8_t((command >> 8) & 0x00FF);
	// while (1) {
	// 	uint8_t byte = CMD_ADDR_SUBCMD_LOW;
	// 	uint8_t resp[2] = {};
	// 	transfer(&byte, 1, resp, 2);

	// 	if ((resp[0] == low_byte) && (resp[1] == high_byte)) {
	// 		break;
	// 	}
	// }

	uint8_t addr = CMD_ADDR_TRANSFER_BUFFER;
	int ret = transfer(&addr, 1, buf, length);

	// int ret = transfer(&addr, 1, nullptr, 0);
	// ret |= transfer(nullptr, 0, buf, length);

	// uint8_t tx[2] = {addr, 0x32};
	// int ret = transfer(tx, 2, buf, length);
	// int ret = transfer(tx, 2, nullptr, 0);
	// ret |= transfer(nullptr, 0, buf, length);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return ret;
}


// int BQ76952::readReg(uint8_t addr, uint8_t *buf, size_t len)
// {
//  return transfer(&addr, 1, buf, len);
// }

// int BQ76952::writeReg(uint8_t addr, uint8_t *buf, size_t len)
// {
//  uint8_t buffer[len + 1];
//  buffer[0] = addr;
//  memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
//  return transfer(buffer, len + 1, nullptr, 0);
// }

void BQ76952::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1:
		{
			enter_config_update_mode();
			sub_command(CMD_OTP_WR_CHECK);
			px4_usleep(5_ms);
			uint16_t status = sub_command_response8(0);

			if (status & (1 << 7)) {
				PX4_INFO("OTP writes enabled: %x", status);
			} else {
				PX4_INFO("OTP writes disabled: %x", status);
			}

			uint8_t buf[2] = {};
			direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
			uint16_t battery_status = (buf[1] << 8) + buf[0];
			PX4_INFO("battery status: 0x%x", battery_status);
			// 0000 1001 1000 0101
			if (battery_status & (1 << 7)) {
				PX4_INFO("OTP writes blocked due to voltage/temperature");
			}

			exit_config_update_mode();
			break;
		}
		case 2:
		{
			read_manu_data();
			break;
		}
		case 3:
		{
			PX4_INFO("Trying to write MANU_DATA");
			enter_config_update_mode();
			sub_command(CMD_OTP_WR_CHECK);
			px4_usleep(5_ms);
			uint16_t status = sub_command_response8(0);
			if (status & (1 << 7)) {
				const char* str = "this is a test";
				PX4_INFO("Writing %s", str);
				sub_command(CMD_MANU_DATA, (void*)str, sizeof(str));
				px4_usleep(50_ms);
			} else {
				PX4_INFO("OTP writes disabled");
			}

			exit_config_update_mode();
			break;
		}
		case 4:
		{
			sub_command(CMD_MFG_STATUS);
			px4_usleep(5_ms);
			uint16_t status = sub_command_response16(0);
			PX4_INFO("mfg status: 0x%x", status);
			break;
		}
		default:
			break;
	}
}

extern "C" int bms_main(int argc, char *argv[])
{
	using ThisDriver = BQ76952;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x08;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DEVTYPE_BQ76952);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "otp_check")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "read_manu")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "write_manu")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "mfg")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
