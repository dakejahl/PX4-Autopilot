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
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{}

BQ76952::~BQ76952()
{
	ScheduleClear();
	perf_free(_cycle_perf);
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

	perf_begin(_cycle_perf);

	battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	int ret = PX4_OK;

	// Read stack voltage
	int16_t stack_voltage = {};
	ret |= direct_command(CMD_READ_STACK_VOLTAGE, &stack_voltage, sizeof(stack_voltage));
	px4_usleep(50);
	battery_status.voltage_v = stack_voltage / 100.0f;
	battery_status.voltage_filtered_v = battery_status.voltage_v; // TODO: filter
	// PX4_INFO("stack_voltage: %f", double(battery_status.voltage_v));

	// Read current (centi-amp resolution 327amps +/-)
	int16_t current = {};
	ret |= direct_command(CMD_READ_CC2_CURRENT, &current, sizeof(current));
	px4_usleep(50);
	battery_status.current_a = current / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a; // TODO: filter
	// PX4_INFO("current: %f", double(battery_status.current_a));

	// Read temperature
	int16_t temperature = {};
	ret |= direct_command(CMD_READ_CFETOFF_TEMP, &temperature, sizeof(temperature));
	battery_status.temperature = (temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C
	// PX4_INFO("temperature: %f", double(battery_status.temperature));

	// Read cell voltages
	int16_t cell_voltages_mv[12] = {};
	ret |= direct_command(CMD_READ_CELL_VOLTAGE, &cell_voltages_mv, sizeof(cell_voltages_mv));
	px4_usleep(50);

	for (size_t i = 0; i < sizeof(cell_voltages_mv) / sizeof(cell_voltages_mv[0]); i++) {
		battery_status.voltage_cell_v[i] = cell_voltages_mv[i] / 1000.0f;
		// PX4_INFO("cellv%zu: %f", i, double(battery_status.voltage_cell_v[i]));
	}

	// Read fault (Battery Status)


	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	// Publish to uORB
	_battery_status_pub.publish(battery_status);

	perf_end(_cycle_perf);
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

	if (probe() != PX4_OK) {
		PX4_ERR("probe failed");
		return PX4_ERROR;
	}

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

	// enable_fets();
	// disable_fets();

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

void BQ76952::print_mfg_status_flags(uint16_t status)
{
	PX4_INFO("mfg status: 0x%x", status);
}

void BQ76952::enable_fets()
{
	sub_command(CMD_FET_ENABLE, 0, 0);
	px4_usleep(500);
	sub_command(CMD_ALL_FETS_ON, 0, 0);
	px4_usleep(500);
}

void BQ76952::disable_fets()
{
	sub_command(CMD_FET_ENABLE, 0, 0);
	px4_usleep(500);
	sub_command(CMD_ALL_FETS_OFF, 0, 0);
	px4_usleep(500);
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
		uint8_t buf[5] = {};
		buf[0] = CMD_ADDR_SUBCMD_LOW;
		buf[1] = uint8_t(addr & 0x00FF);
		buf[2] = uint8_t((addr >> 8) & 0x00FF);
		buf[3] = data;

		transfer(buf, sizeof(buf), nullptr, 0);
		for (size_t i = 1; i < 3 + 2; i++) {
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
		for (size_t i = 1; i < 3 + 2; i++) {
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
