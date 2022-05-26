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

#include "BQ76952.hpp"

BQ76952::BQ76952() :
	I2C(DRV_DEVTYPE_BQ76952, "BQ76952", 1, 0x08, 400000),
	_comms_errors(perf_alloc(PC_COUNT, "BQ76952: comm errors"))
{}

BQ76952::~BQ76952()
{
	perf_free(_comms_errors);
}

int BQ76952::init()
{
	PX4_INFO("Initializing BQ76952");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	return PX4_OK;
}

int BQ76952::configure_settings()
{
	int ret = PX4_OK;
	ret = write_memory8(ADDR_REG12_CONFIG, 0x0d); // Enable 3.3V for REG1
	if (ret != PX4_OK) {
		PX4_ERR("writing REG12 failed");
		return PX4_ERROR;
	}

	ret = write_memory8(ADDR_REG0, 0x01); // Enable the pre-regulator to turn on bq34z100
	if (ret != PX4_OK) {
		PX4_ERR("writing REG0 failed");
		return PX4_ERROR;
	}

	ret = write_memory8(ADDR_TS1_CONFIG, 0x0f); // FET temp monitoring
	if (ret != PX4_OK) {
		PX4_ERR("writing TS1_CONFIG failed");
		return PX4_ERROR;
	}

	ret = write_memory8(ADDR_TS3_CONFIG, 0x07); // Cell temp monitoring
	if (ret != PX4_OK) {
		PX4_ERR("writing TS3_CONFIG failed");
		return PX4_ERROR;
	}
	ret = write_memory8(ADDR_DA_CONFIG, 0x06); // Centi-volt and centi-amp
	if (ret != PX4_OK) {
		PX4_ERR("writing DA_CONFIG failed");
		return PX4_ERROR;
	}
	return PX4_OK;
}

void BQ76952::configure_fets()
{
	// Check Manufacturing Status register and set FET mode if needed
	sub_command(CMD_MFG_STATUS);
	px4_usleep(5_ms);
	uint16_t status = sub_command_response16(0);
	PX4_INFO("mfg status: 0x%x", status);

	// Set FET "normal mode" if not already set
	if (!(status & (1 << 4))) {
		PX4_INFO("Enabling FET normal mode");
		sub_command(CMD_FET_ENABLE);
	}

	// Set FET Protection action
	configure_protections_fet_action();
}

uint16_t BQ76952::mfg_status()
{
	sub_command(CMD_MFG_STATUS);
	px4_usleep(5_ms);
	return sub_command_response16(0);
}

uint32_t BQ76952::status_flags()
{
	uint32_t status_flags = {};

	// SAFETY ALERT/STATUS A
	{
		int ret = PX4_OK;
		uint8_t safety_status_a = {};
		ret |= direct_command(CMD_SAFETY_STATUS_A, &safety_status_a, sizeof(safety_status_a));

		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t under_volt_mask = (1 << 2);
		if (safety_status_a & under_volt_mask) {
			status_flags |= STATUS_FLAG_FAULT_UNDER_VOLT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_volt_mask = (1 << 3);
		if (safety_status_a & over_volt_mask) {
			status_flags |= STATUS_FLAG_FAULT_OVER_VOLT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_current_mask = (1 << 4) | (1 << 5) | (1 << 6);
		if (safety_status_a & over_current_mask) {
			status_flags |= STATUS_FLAG_FAULT_OVER_CURRENT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t short_circuit_mask = (1 << 7);
		if (safety_status_a & short_circuit_mask) {
			status_flags |= STATUS_FLAG_FAULT_SHORT_CIRCUIT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	// SAFETY ALERT/STATUS B
	{
		int ret = PX4_OK;
		uint8_t safety_status_b = {};
		ret |= direct_command(CMD_SAFETY_STATUS_B, &safety_status_b, sizeof(safety_status_b));


		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_temp_mask = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4);
		if (safety_status_b & over_temp_mask) {
			status_flags |= STATUS_FLAG_FAULT_OVER_TEMP;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t under_temp_mask = (1 << 0) | (1 << 1) | (1 << 2);
		if (safety_status_b & under_temp_mask) {
			status_flags |= STATUS_FLAG_FAULT_UNDER_TEMP;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	// SAFETY ALERT/STATUS C
	{
		int ret = PX4_OK;
		uint8_t safety_status_c = {};
		ret |= direct_command(CMD_SAFETY_STATUS_C, &safety_status_c, sizeof(safety_status_c));

		if (ret != PX4_OK) {
			status_flags |= STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_current_mask = (1 << 7) | (1 << 5);
		if (safety_status_c & over_current_mask) {
			status_flags |= STATUS_FLAG_FAULT_OVER_CURRENT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t short_circuit_mask = (1 << 6);
		if (safety_status_c & short_circuit_mask) {
			status_flags |= STATUS_FLAG_FAULT_SHORT_CIRCUIT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_volt_mask = (1 << 4);
		if (safety_status_c & over_volt_mask) {
			status_flags |= STATUS_FLAG_FAULT_OVER_VOLT;
			status_flags |= STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	return status_flags;
}

void BQ76952::manu_data()
{
	PX4_INFO("manu_data");
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

void BQ76952::enable_protections()
{
	PX4_INFO("Enabling protections");
	// ADDR_PROTECTION_CONFIG -- default looks good
	// SCDL_CURR_RECOV -- 1 = SCDL recovers when current is greater than or equal to Protections:SCDL:RecoveryTime.
	// OCDL_CURR_RECOV -- 1 = OCDL recovers when current is greater than or equal to Protections:OCDL:RecoveryTime
	// PF_OTP -- If this bit is not set, Permanent Failure status will be lost on any reset

	//  The individual protections can be enabled by setting the related Settings:Protection:Enabled Protections
	//  A – C configuration registers.

	// Settings:Protection:Enabled Protections A
	{
		uint8_t byte = {};

		byte |= 1 << 7; 	// 7 SCD 1 Short Circuit in Discharge Protection
							// TODO: set the threshold

		// byte |= 1 << 6; 	// 6 OCD2 0 Overcurrent in Discharge 2nd Tier Protection

		byte |= 1 << 5; 	// 5 OCD1 0 Overcurrent in Discharge 1st Tier Protection
							// TODO: set the threshold

		byte |= 1 << 4; 	// 4 OCC 0 Overcurrent in Charge Protection
							// TODO: set the threshold

		byte |= 1 << 3; 	// 3 COV 1 Cell Overvoltage Protection
							// TODO: set the threshold

		byte |= 1 << 2; 	// 2 CUV 0 Cell Undervoltage Protection
							// TODO: set the threshold

		write_memory8(ADDR_PROTECTIONS_A, byte);
	}

	// Settings:Protection:Enabled Protections B
	{
		uint8_t byte = {};

		byte |= 1 << 7; 	// 7 OTF 0 FET Overtemperature

		byte |= 1 << 6; 	// 6 OTINT 0 Internal Overtemperature

		byte |= 1 << 5; 	// 5 OTD 0 Overtemperature in Discharge

		byte |= 1 << 4; 	// 4 OTC 0 Overtemperature in Charge

		byte |= 1 << 2; 	// 2 UTINT 0 Internal Undertemperature

		byte |= 1 << 1; 	// 1 UTD 0 Undertemperature in Discharge

		byte |= 1 << 0; 	// 0 UTC 0 Undertemperature in Charge

		write_memory8(ADDR_PROTECTIONS_B, byte);
	}

	// Settings:Protection:Enabled Protections C
	{
		uint8_t byte = {};

		// byte |= 1 << 7; // 7 OCD3 0 Overcurrent in Discharge 3rd Tier Protection

		// byte |= 1 << 6; 	// 6 SCDL 0 Short Circuit in Discharge Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 5; 	// 5 OCDL 0 Overcurrent in Discharge Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 4; 	// 4 COVL 0 Cell Overvoltage Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 2; 	// 2 PTO 0 Precharge Timeout
							// TODO: do we want to use this feature?

		// byte |= 1 << 1; // 1 HWDF 0 Host Watchdog Fault

		write_memory8(ADDR_PROTECTIONS_C, byte);
	}
}

void BQ76952::disable_protections()
{
	// We disable all protections except for

	// Cell overvoltage should probably always be on.
	// Over temperature in charge we always want on
	// Undertemperature in charge we always want on
	PX4_INFO("Disabling protections");
	// {
	// 	uint8_t byte = {};
	// 	byte |= 1 << 4; // Overcurrent in charge protection stays on
	// 	write_memory8(ADDR_PROTECTIONS_A, byte);
	// }
	// {
	// 	uint8_t byte = {};
	// 	byte |= 1 << 4; // Overcurrent in charge protection stays on
	// 	write_memory8(ADDR_PROTECTIONS_B, byte);
	// }
	// {
	// 	uint8_t byte = {};
	// 	byte |= 1 << 4; // Overcurrent in charge protection stays on
	// 	write_memory8(ADDR_PROTECTIONS_C, byte);
	// }
}

void BQ76952::configure_protections_fet_action()
{
	PX4_INFO("Configuring FET protection actions");

	// CHG FET Protections A
	{
		uint8_t byte = {};

		// 7 SCD 1 Short Circuit in Discharge Protection
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 7);

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
		// TODO: set threshold

		// 6 OTINT 1 Internal Overtemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 6);
		// TODO: set threshold

		// 4 OTC 1 Overtemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);
		// TODO: set threshold

		// 2 UTINT 1 Internal Undertemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 2);
		// TODO: set threshold

		// 0 UTC 1 Undertemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);
		// TODO: set threshold

		write_memory8(ADDR_CHG_FET_Protections_B, byte);
	}

	// CHG FET Protections C
	{
		uint8_t byte = {};

		// 6 SCDL 1 Short Circuit in Discharge Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (6 << 0);

		// 4 COVL 1 Cell Overvoltage Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (4 << 0);

		// 2 PTO 1 Precharge Timeout
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (2 << 0);

		// 1 HWDF 1 Host Watchdog Fault
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 0);

		// TODO: do we want to use these features? ^

		write_memory8(ADDR_CHG_FET_Protections_C, byte);
	}

	// TODO:
	// DSG FET Protections A
	{
		uint8_t byte = {};
		write_memory8(ADDR_DSG_FET_Protections_A, byte);
	}
}


void BQ76952::enable_fets()
{
	PX4_INFO("Enabling FETs");
	sub_command(CMD_ALL_FETS_ON);
	px4_usleep(5_ms);
}

void BQ76952::disable_fets()
{
	PX4_INFO("Disabling FETs");
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
	PX4_INFO("Entering CONFIG_UPDATE mode");
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

	PX4_INFO("FAILED to enter CONFIG_UPDATE mode");
	return PX4_ERROR;
}

int BQ76952::exit_config_update_mode()
{
	PX4_INFO("Exiting CONFIG_UPDATE mode");

	sub_command(CMD_EXIT_CFG_UPDATE);
	px4_usleep(2_ms); // 1000us time to complete operation

	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	px4_usleep(5_ms);
	if (!(buf[0] & 0x01)) {
		return PX4_OK;
	}

	PX4_INFO("FAILED to exit CONFIG_UPDATE mode");
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

int BQ76952::probe()
{
	uint8_t val = {};
	int ret = transfer(&val, sizeof(val), nullptr, 0);
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	return ret;
}
