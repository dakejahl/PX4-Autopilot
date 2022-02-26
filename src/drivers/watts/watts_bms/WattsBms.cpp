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

#include "WattsBms.hpp"

WattsBms::WattsBms(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{}

WattsBms::~WattsBms()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

void WattsBms::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void WattsBms::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("exiting");
		return;
	}

	perf_begin(_cycle_perf);

	_battery_status_report.timestamp = hrt_absolute_time();

	// TODO: Do stuff
	uint8_t buf[2] = {};
	int ret = direct_command(CMD_READ_VOLTAGE_STACK, buf, sizeof(buf));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	// float stack_voltage = (buf[1] << 8) | buf[0];
	// PX4_INFO("stack_voltage: %f", double(stack_voltage / 100)); // units of 0.01v

	perf_end(_cycle_perf);
}

void WattsBms::print_usage()
{
	PRINT_MODULE_USAGE_NAME("watts_bms", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void WattsBms::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

int WattsBms::init()
{
	PX4_INFO("Initializing WattsBms");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	// TODO: test that this works
	if (I2C::probe() != PX4_OK) {
		return PX4_ERROR;
	}

	// First disable LDOs if they are enabled
	uint8_t value = 0b00001100;
	sub_command(CMD_REG12_CONTROL, &value, 1);
	usleep(900); // 450us ttco

	///// SET SETTINGS INTO PERMANENT MEMORY /////


	// Set REG1 voltage to 3.3v
	value = REG1_ENABLE_3v3;
	ret = write_memory(ADDR_REG12_CONFIG, &value, 1);
	if (ret != PX4_OK) {
		PX4_ERR("configuring REG1 failed");
	}

	// Enable regulator(s)
	value = 0x01;
	ret = write_memory(ADDR_REG0, &value, 1); // Enable the other bq
	if (ret != PX4_OK) {
		PX4_ERR("enabling regulator failed");
	}

	// Enable LDO at REG1
	value = 0b00001101;
	sub_command(CMD_REG12_CONTROL, &value, 1);
	usleep(900); // 450us ttco

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int WattsBms::enter_config_update_mode()
{
	// Enter config udpate mode if not already in it and report status
	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	if (!(buf[0] & 0x01)) {
		sub_command(CMD_SET_CFGUPDATE, nullptr, 0);
		usleep(4000); // 2000us time to complete operation
		direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
		if (buf[0] & 0x01) {
			return PX4_OK;
		}
	} else {
		// Already in config update mode
		return PX4_OK;
	}

	return PX4_ERROR;
}

int WattsBms::exit_config_update_mode()
{
	sub_command(CMD_EXIT_CFG_UPDATE, nullptr, 0);
	usleep(2000); // 1000us time to complete operation

	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	if (!(buf[0] & 0x01)) {
		return PX4_OK;
	}
	return PX4_ERROR;
}

int WattsBms::write_memory(uint16_t addr, uint8_t* tx_buf, size_t tx_len)
{
	PX4_INFO_RAW("Writing to %x --> ", addr);

	for (size_t i = 0; i < tx_len; i++) {
		PX4_INFO_RAW("%x", tx_buf[i]);
	}
	PX4_INFO_RAW("\n");

	int ret = enter_config_update_mode();
	if (ret != PX4_OK) {
		PX4_ERR("failed to write memory");
		return PX4_ERROR;
	}

	// The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
	// number of bytes used in the transfer buffer, then the result is bitwise inverted
	uint8_t checksum = 0;
	uint8_t transer_length = 0; // See pg 13 of technical reference
	// Send the data to the transfer buffer
	{
		uint8_t buf[3 + tx_len] = {};
		buf[0] = CMD_ADDR_SUBCMD_LOW;
		buf[1] = uint8_t(addr & 0x00FF);
		buf[2] = uint8_t((addr >> 8) & 0x00FF);
		memcpy(buf + 3, tx_buf, tx_len);
		transfer(buf, tx_len + 2, nullptr, 0);
		transer_length += tx_len + 2;

		for (size_t i = 1; i < 3 + tx_len; i++) {
			checksum += buf[i];
		}
		checksum = ~checksum;
	}

	// Send checksum and length
	{
		uint8_t buf[3] = {};
		buf[0] = CMD_ADDR_RESP_CHKSUM;
		buf[1] = checksum;
		buf[2] = transer_length + 2;
		transfer(buf, sizeof(buf), nullptr, 0);
	}

	exit_config_update_mode();

	return PX4_OK;
}

int WattsBms::direct_command(uint8_t command, uint8_t* rx_buf, size_t rx_len)
{
	return transfer(&command, 1, rx_buf, rx_len);
}

int WattsBms::sub_command(uint16_t command, uint8_t* tx_buf, size_t tx_len)
{
	uint8_t buf[3 + tx_len] = {};
	buf[0] = 0x3E;
	buf[1] = uint8_t(command & 0x00FF);
	buf[2] = uint8_t((command >> 8) & 0x00FF);
	memcpy(buf + 3, tx_buf, tx_len);

	return transfer(buf, sizeof(buf), nullptr, 0);
}

uint16_t WattsBms::sub_command_response16(uint8_t offset)
{
	uint8_t addr = CMD_ADDR_TRANSFER_BUFFER + offset;
	uint8_t buf[2] = {};

	transfer(&addr, 1, buf, sizeof(buf));

	return (buf[0] << 8) | buf[1];
}


// int WattsBms::readReg(uint8_t addr, uint8_t *buf, size_t len)
// {
// 	return transfer(&addr, 1, buf, len);
// }

// int WattsBms::writeReg(uint8_t addr, uint8_t *buf, size_t len)
// {
// 	uint8_t buffer[len + 1];
// 	buffer[0] = addr;
// 	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
// 	return transfer(buffer, len + 1, nullptr, 0);
// }
