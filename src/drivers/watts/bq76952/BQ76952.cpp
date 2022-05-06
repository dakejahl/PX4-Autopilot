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
}

void BQ76952::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void BQ76952::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("exiting");
		return;
	}

	if (_shutting_down) {
		// Wait until button is released
		if (px4_arch_gpioread(GPIO_N_BTN)) {
			shutdown();
		}
		return;
	}

	perf_begin(_cycle_perf);

	update_params();

	// Detect button presses and handle corresponding behavior
	handle_button();
	// If drawing a very low amount of power for some amount of time, automatically turn pack off
	handle_idle_current_detection();
	// Automatically enable/disable protections
	handle_automatic_protections();

	// TODO: check if we are armed/disarmed and enable/disable protections
	bool current_above_threshold = false;
	if (current_above_threshold) {

	}

	collect_and_publish();

	perf_end(_cycle_perf);
}

void BQ76952::collect_and_publish()
{
	watts_battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	int ret = PX4_OK;

	// Read temperature
	int16_t temperature = {};
	ret |= direct_command(CMD_READ_CFETOFF_TEMP, &temperature, sizeof(temperature));
	battery_status.temperature = ((float)temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C
	// PX4_INFO("temperature: %f", double(battery_status.temperature));

	// Read current (centi-amp resolution 327amps +/-)
	int16_t current = {};
	ret |= direct_command(CMD_READ_CC2_CURRENT, &current, sizeof(current));
	px4_usleep(50);
	battery_status.current = (float)current / 100.0f;
	// PX4_INFO("current: %f", double(battery_status.current_a));

	// Read stack voltage
	int16_t stack_voltage = {};
	ret |= direct_command(CMD_READ_STACK_VOLTAGE, &stack_voltage, sizeof(stack_voltage));
	px4_usleep(50);
	battery_status.voltage = (float)stack_voltage / 100.0f;

	// ignore capacity consumed

	// Read capacity remaining
	battery_status.capacity_remaining = _bq34->read_remaining_capacity();
	// PX4_INFO("capacity_remaining (mAh) %lu", capacity_remaining);

	// Read cell voltages
	int16_t cell_voltages_mv[12] = {};
	ret |= direct_command(CMD_READ_CELL_VOLTAGE, &cell_voltages_mv, sizeof(cell_voltages_mv));
	px4_usleep(50);

	for (size_t i = 0; i < sizeof(cell_voltages_mv) / sizeof(cell_voltages_mv[0]); i++) {
		battery_status.cell_voltages[i] = cell_voltages_mv[i] / 1000.0f;
		// PX4_INFO("cellv%zu: %f", i, double(battery_status.voltage_cell_v[i]));
	}

	// Read design capacity
	battery_status.design_capacity = _bq34->read_design_capacity();
	// Read actual_capacity
	battery_status.actual_capacity = _bq34->read_full_charge_capacity();
	// Read cycle count
	battery_status.cycle_count = _bq34->read_cycle_count();
	// Read state of health
	battery_status.state_of_health = _bq34->read_state_of_health();

	// Read bq76 faults (0x12 Battery Status())
	// uint32_t status_flags = {};

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	// Publish to uORB
	_battery_status_pub.publish(battery_status);
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
			PX4_INFO("Current exceeds PROTECT_CURRENT (%2.2f), disabling protections", double(protect_current));
			_protections_enabled = false;
		}
	} else {
		if (!_protections_enabled) {
			// Enable protections
			PX4_INFO("Current is below PROTECT_CURRENT (%2.2f), enabling protections", double(protect_current));
			_protections_enabled = true;
		}
	}
}

void BQ76952::handle_idle_current_detection()
{
	int32_t idle_timeout = _param_idle_timeout.get();
	if ( idle_timeout == 0) {
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

		if (now > _idle_start_time + (hrt_abstime)idle_timeout) {
			PX4_INFO("Battery has been idle for %lu, shutting down", idle_timeout);
			// TODO: actually shut down
			_shutdown_pub.publish(shutdown_s{});
			_shutting_down = true;
		}

	} else {
		_below_idle_current = false;
	}
	// 0 disables timeout
}

void BQ76952::handle_button()
{
	if (!_booted) {
		bool held = check_button_held();

		if (held) {
			PX4_INFO("Button was held, turning on FETs");
			_booted = true;
			enable_fets();
		}

		// Check if 5 seconds has elapsed, power off
		if (hrt_absolute_time() > 5_s) {
			PX4_INFO("Button not held");
			_shutdown_pub.publish(shutdown_s{});
			_shutting_down = true;
		}

	} else {
		bool held = check_button_held();

		if (held) {
			PX4_INFO("Button was held, disabling FETs");
			// Notify shutdown
			_shutdown_pub.publish(shutdown_s{});
			_shutting_down = true;
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
		_button_pressed = false;
	}

	return false;
}

void BQ76952::shutdown()
{
	disable_fets();
	PX4_INFO("Good bye!");
	px4_usleep(50000);
	stm32_gpiowrite(GPIO_PWR_EN, false);
	px4_usleep(50000);
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
	return transfer(&val, sizeof(val), nullptr, 0);
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

	// First disable LDOs if they are enabled
	uint8_t value = 0b00001100;
	sub_command(CMD_REG12_CONTROL, &value, sizeof(value));
	px4_usleep(450);

	///// WRITE SETTINGS INTO PERMANENT MEMORY /////

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

	// Enable LDO at REG1
	value = 0b00001101;
	sub_command(CMD_REG12_CONTROL, &value, sizeof(value));
	px4_usleep(450);

	// Check Manufacturing Status register
	sub_command(CMD_MFG_STATUS, 0, 0);
	px4_usleep(605);
	uint16_t mfg_status_flags = sub_command_response16(0);
	print_mfg_status_flags(mfg_status_flags);

	// Configure protections
	disable_protections();

	configure_protections();
	// enable_protections();
	disable_protections();

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

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

void BQ76952::configure_protections()
{
	// ADDR_PROTECTION_CONFIG
	// SCDL_CURR_RECOV -- 1 = SCDL recovers when current is greater than or equal to Protections:SCDL:RecoveryTime.
	// OCDL_CURR_RECOV -- 1 = OCDL recovers when current is greater than or equal to Protections:OCDL:RecoveryTime
	// PF_OTP -- If this bit is not set, Permanent Failure status will be lost on any reset

	// 	The individual protections can be enabled by setting the related Settings:Protection:Enabled Protections
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
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 7);

		// 4 OCC 1 Overcurrent in Charge Protection
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);

		// 3 COV 1 Cell Overvoltage Protection
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 3);

		write_memory8(ADDR_CHG_FET_Protections_A, byte);
	}

	// CHG FET Protections B
	{
		uint8_t byte = {};

		// 7 OTF 1 FET Overtemperature
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 7);

		// 6 OTINT 1 Internal Overtemperature
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 6);

		// 4 OTC 1 Overtemperature in Charge
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);

		// 2 UTINT 1 Internal Undertemperature
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 2);

		// 0 UTC 1 Undertemperature in Charge
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);

		write_memory8(ADDR_CHG_FET_Protections_B, byte);
	}

	// CHG FET Protections C
	{
		uint8_t byte = {};

		// 6 SCDL 1 Short Circuit in Discharge Latch
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (6 << 0);

		// 4 COVL 1 Cell Overvoltage Latch
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (4 << 0);

		// 2 PTO 1 Precharge Timeout
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (2 << 0);

		// 1 HWDF 1 Host Watchdog Fault
		// 		0 = CHG FET is not disabled when protection is triggered.
		// 		1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);

		write_memory8(ADDR_CHG_FET_Protections_C, byte);
	}
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

void BQ76952::print_mfg_status_flags(uint16_t status)
{
	PX4_INFO("mfg status: 0x%x", status);
}

void BQ76952::enable_fets()
{
	px4_usleep(5000);
	sub_command(CMD_FET_ENABLE, 0, 0);
	px4_usleep(5000);
	sub_command(CMD_ALL_FETS_ON, 0, 0);
	px4_usleep(5000);
}

void BQ76952::disable_fets()
{
	sub_command(CMD_FET_ENABLE, 0, 0);
	px4_usleep(500);
	sub_command(CMD_ALL_FETS_OFF, 0, 0);
	px4_usleep(500);
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

		transfer(buf, sizeof(buf), nullptr, 0);
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
		transfer(buf, sizeof(buf), nullptr, 0);
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

		transfer(buf, sizeof(buf), nullptr, 0);
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
		transfer(buf, sizeof(buf), nullptr, 0);
	}

	exit_config_update_mode();

	return PX4_OK;
}

int BQ76952::enter_config_update_mode()
{
	// Enter config udpate mode if not already in it and report status
	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	px4_usleep(50);
	if (!(buf[0] & 0x01)) {
		sub_command(CMD_SET_CFGUPDATE, nullptr, 0);
		px4_usleep(4000); // 2000us time to complete operation
		direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
		px4_usleep(50);
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
	sub_command(CMD_EXIT_CFG_UPDATE, nullptr, 0);
	px4_usleep(2000); // 1000us time to complete operation

	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	px4_usleep(50);
	if (!(buf[0] & 0x01)) {
		return PX4_OK;
	}
	return PX4_ERROR;
}

int BQ76952::direct_command(uint8_t command, void* rx_buf, size_t rx_len)
{
	return transfer(&command, 1, (uint8_t*)rx_buf, rx_len);
}

int BQ76952::sub_command(uint16_t command, void* tx_buf, size_t tx_len)
{
	uint8_t buf[3 + tx_len] = {};
	buf[0] = 0x3E;
	buf[1] = uint8_t(command & 0x00FF);
	buf[2] = uint8_t((command >> 8) & 0x00FF);
	memcpy(buf + 3, (uint8_t*)tx_buf, tx_len);

	return transfer(buf, sizeof(buf), nullptr, 0);
}

uint16_t BQ76952::sub_command_response16(uint8_t offset)
{
	uint8_t addr = CMD_ADDR_TRANSFER_BUFFER + offset;
	uint8_t buf[2] = {};

	transfer(&addr, 1, buf, sizeof(buf));

	return (buf[0] << 8) | buf[1];
}


// int BQ76952::readReg(uint8_t addr, uint8_t *buf, size_t len)
// {
// 	return transfer(&addr, 1, buf, len);
// }

// int BQ76952::writeReg(uint8_t addr, uint8_t *buf, size_t len)
// {
// 	uint8_t buffer[len + 1];
// 	buffer[0] = addr;
// 	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
// 	return transfer(buffer, len + 1, nullptr, 0);
// }

void BQ76952::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1:
			PX4_INFO("custom command 1");
			break;
		default:
			break;
	}
}

extern "C" int bq76952_main(int argc, char *argv[])
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

	if (!strcmp(verb, "custom_command")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
