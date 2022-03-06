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
#include <drivers/device/i2c.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>
#include <uORB/Publication.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

// Direct commands(1 byte)
#define CMD_BATTERY_STATUS 		0x12
#define CMD_READ_VOLTAGE_STACK 	0x34

// Subcommands (2 bytes)
#define CMD_ADDR_SUBCMD_LOW    	0x3E

#define CMD_ENTER_CFG_UPDATE 	0x0034 // The device will then automatically disable the protection FETs if they are enabled.

#define CMD_SET_CFGUPDATE 		0x0090 // The device will then automatically disable the protection FETs if they are enabled.
#define CMD_EXIT_CFG_UPDATE 	0x0092 // The device will then automatically disable the protection FETs if they are enabled.

#define CMD_REG12_CONTROL 		0x0098

// 7-bit Command Addresses
#define CMD_ADDR_TRANSFER_BUFFER 0x40 // 32-byte transfer buffer
#define CMD_ADDR_RESP_CHKSUM     0x60

// Memory Addresses
#define ADDR_REG12_CONFIG 	0x9236
#define ADDR_REG0 			0x9237

// Register Bitmasks
#define REG1_ENABLE_3v3 0b00001101

class BQ76952 : public device::I2C, public I2CSPIDriver<BQ76952>
{
public:
	BQ76952(const I2CSPIDriverConfig &config);
	~BQ76952() override;

	int init() override;

	static void print_usage();

	void RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:
	int enter_config_update_mode();
	int exit_config_update_mode();

	uORB::Publication<battery_status_s>	_battery_status_pub {ORB_ID(battery_status)};

	static const hrt_abstime	SAMPLE_INTERVAL {500_ms};

	battery_status_s _battery_status_report {};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

    int direct_command(uint8_t command, uint8_t* rx_buf, size_t rx_len);

	int sub_command(uint16_t command, uint8_t* tx_buf, size_t tx_len);
	uint16_t sub_command_response16(uint8_t offset);

    int write_memory8(uint16_t addr, uint8_t data);
    int write_memory16(uint16_t addr, uint16_t data);

	// int readReg(uint8_t addr, uint8_t *buf, size_t len);
	// int writeReg(uint8_t addr, uint8_t *buf, size_t len);

};
