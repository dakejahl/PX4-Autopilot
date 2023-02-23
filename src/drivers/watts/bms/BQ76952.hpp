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

#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <lib/geo/geo.h>

using namespace time_literals;

// Direct commands(1 byte)
#define CMD_SAFETY_ALERT_A     0x02
#define CMD_SAFETY_STATUS_A     0x03
#define CMD_SAFETY_ALERT_B     0x04
#define CMD_SAFETY_STATUS_B     0x05
#define CMD_SAFETY_ALERT_C     0x06
#define CMD_SAFETY_STATUS_C     0x07

#define CMD_BATTERY_STATUS      0x12
#define CMD_READ_CELL_VOLTAGE   0x14
#define CMD_READ_STACK_VOLTAGE  0x34
#define CMD_READ_PACK_PIN_VOLTAGE  0x36
#define CMD_READ_CC2_CURRENT    0x3A
#define CMD_READ_CFETOFF_TEMP   0x6A

#define CMD_READ_TS1_TEMP   	0x70 // units of 0.1K (signed 2 bytes)
#define CMD_READ_TS3_TEMP   	0x74 // units of 0.1K (signed 2 bytes)

// Subcommands (2 bytes)
#define CMD_ADDR_SUBCMD_LOW     0x3E

#define CMD_DEVICE_NUMBER       0x0001
#define CMD_FET_ENABLE          0x0022
#define CMD_ENTER_CFG_UPDATE    0x0034 // The device will then automatically disable the protection FETs if they are enabled.
#define CMD_MFG_STATUS          0x0057 // pg 119
#define CMD_MANU_DATA          	0x0070
#define CMD_DASTATUS_5          0x0075 // Used to get CC3 and CC2 currents, cell and fet temperatures

#define CMD_SET_CFGUPDATE       0x0090 // The device will then automatically disable the protection FETs if they are enabled.
#define CMD_EXIT_CFG_UPDATE     0x0092 // The device will then automatically disable the protection FETs if they are enabled.
#define CMD_ALL_FETS_OFF        0x0095
#define CMD_ALL_FETS_ON         0x0096
#define CMD_REG12_CONTROL       0x0098
#define CMD_OTP_WR_CHECK        0x00A0

// 7-bit Command Addresses
#define CMD_ADDR_TRANSFER_BUFFER 0x40 // 32-byte transfer buffer
#define CMD_ADDR_RESP_CHKSUM     0x60

// Memory Addresses
#define ADDR_CELL_1_GAIN 	0x9180
#define ADDR_REG12_CONFIG   0x9236
#define ADDR_REG0           0x9237
#define ADDR_DA_CONFIG      0x9303

#define ADDR_PROTECTION_CONFIG  0x925F // 2 byte
#define ADDR_PROTECTIONS_A      0x9261 // 1 byte
#define ADDR_PROTECTIONS_B      0x9262 // 1 byte
#define ADDR_PROTECTIONS_C      0x9263 // 1 byte

#define ADDR_TS1_CONFIG      	0x92FD // 1 byte
#define ADDR_TS3_CONFIG      	0x92FF // 1 byte

#define ADDR_CHG_FET_Protections_A      0x9265 // 1 byte
#define ADDR_CHG_FET_Protections_B      0x9266 // 1 byte
#define ADDR_CHG_FET_Protections_C      0x9267 // 1 byte
#define ADDR_DSG_FET_Protections_A      0x9269 // 1 byte
#define ADDR_DSG_FET_Protections_B      0x926A // 1 byte
#define ADDR_DSG_FET_Protections_C      0x926B // 1 byte

#define ADDR_MFG_STATUS_INIT 0x9343

// Register Bitmasks
#define DA_CONFIG_CENTIVOLT_CENTIAMP 0b00000110

template <typename T>
struct Register {
	const char* name;
	uint16_t address;
	T value;
};

class BQ76952 : public device::I2C
{
public:
	BQ76952();
	~BQ76952() override;

	// Initialize things
	int init();
	int probe() override;

	// Monitor things
	float temperature_cells();
	float temperature_fets();
	float current();
	float bat_voltage();
	float pack_voltage();
	void cell_voltages(float* cells_array, size_t size);
	uint32_t status_flags();
	uint16_t mfg_status();
	uint16_t battery_status();
	uint8_t otp_wr_check();

	// Control things
	void enable_protections();
	void disable_protections();
	void enable_fets();
	void disable_fets();

private:
	// Register Configuration
	int configure_settings();

	void configure_fets();
	void configure_protections_fet_action();

	// Modes
	int enter_config_update_mode();
	int exit_config_update_mode();

	// Command and response
	int direct_command(uint8_t command, void* rx_buf, size_t rx_len);
	int sub_command(uint16_t command, void* tx_buf = nullptr, size_t tx_len = 0);
	uint8_t sub_command_response8(uint8_t offset);
	uint16_t sub_command_response16(uint8_t offset);

	// Memory access
	uint8_t read_memory8(uint16_t addr);
	uint16_t read_memory16(uint16_t addr);

	static const hrt_abstime SAMPLE_INTERVAL{50_ms};

	perf_counter_t _comms_errors{};

	char _manu_data[32]{0};

// TEMPLATE FUNCTIONS
private:
	template <typename T>
	int write_register(Register<T> reg)
	{
		PX4_INFO("Writing 0x%x to %s", (unsigned int)reg.value, reg.name);

		int ret = PX4_OK;
		ret = write_memory<T>(reg.address, reg.value);
		px4_usleep(5_ms);

		if (ret != PX4_OK) {
			PX4_ERR("%s", reg.name);
			return PX4_ERROR;
		}
		return PX4_OK;
	}

	template <typename T>
	int write_memory(uint16_t addr, T data)
	{
		int ret = PX4_OK;

		// See pg 13 of technical reference
		// The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
		// number of bytes used in the transfer buffer, then the result is bitwise inverted
		uint8_t checksum = 0;
		// Send the data
		{
			uint8_t buf[3 + sizeof(data)] = {};
			buf[0] = CMD_ADDR_SUBCMD_LOW;
			buf[1] = uint8_t(addr & 0x00FF);
			buf[2] = uint8_t((addr >> 8) & 0x00FF);
			memcpy(&buf[3], &data, sizeof(data));

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
			buf[2] = 2 + sizeof(data) + 1 + 1; // 2 bytes addr, N data, 1 byte checksum, 1 byte length
			ret |= transfer(buf, sizeof(buf), nullptr, 0);
		}

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
		}

		return PX4_OK;
	}
};
