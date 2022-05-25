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
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/watts_battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/shutdown.h>
#include <uORB/topics/button_pressed.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include "BQ34Z100.hpp"

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
#define CMD_READ_TS3_TEMP   	0x74 // units of 0.1K (signed 2 bytes)

// Subcommands (2 bytes)
#define CMD_ADDR_SUBCMD_LOW     0x3E

#define CMD_FET_ENABLE          0x0022
#define CMD_ENTER_CFG_UPDATE    0x0034 // The device will then automatically disable the protection FETs if they are enabled.
#define CMD_MFG_STATUS          0x0057 // pg 119
#define CMD_MANU_DATA          	0x0070

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

class BQ76952 : public device::I2C, public ModuleParams, public I2CSPIDriver<BQ76952>
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

	int probe() override;
	void update_params(const bool force = false);

	void handle_button_and_boot();
	void handle_idle_current_detection();
	void handle_automatic_protections();

	bool check_button_held();
	void shutdown();

	void collect_and_publish();
	uint32_t get_status_flags();
	void read_manu_data();

	// Register Configuration
	int configure_settings();
	void configure_fets();
	void configure_protections_fet_action();

	// Control things
	void enable_protections();
	void disable_protections();
	int initialize_bq34();
	void enable_fets();
	void disable_fets();

	int enter_config_update_mode();
	int exit_config_update_mode();

	int direct_command(uint8_t command, void* rx_buf, size_t rx_len);
	int sub_command(uint16_t command, void* tx_buf = nullptr, size_t tx_len = 0);

	uint8_t sub_command_response8(uint8_t offset);
	uint16_t sub_command_response16(uint8_t offset);
	int sub_command_response_buffer(uint8_t* buf, size_t length);

	uint8_t read_memory8(uint16_t addr);
	uint16_t read_memory16(uint16_t addr);

	int write_memory8(uint16_t addr, uint8_t data);
	int write_memory16(uint16_t addr, uint16_t data);
	// int readReg(uint8_t addr, uint8_t *buf, size_t len);
	// int writeReg(uint8_t addr, uint8_t *buf, size_t len);
private:
	static const hrt_abstime SAMPLE_INTERVAL{50_ms};

	BQ34Z100* _bq34{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<shutdown_s> _shutdown_pub{ORB_ID(shutdown)};
	uORB::Publication<button_pressed_s>	_button_pressed_pub{ORB_ID(button_pressed)};
	uORB::PublicationMulti<watts_battery_status_s> _battery_status_pub{ORB_ID(watts_battery_status)};

	perf_counter_t _cycle_perf{};
	perf_counter_t _comms_errors{};

	// State variables
	hrt_abstime _pressed_start_time{0};
	bool _button_pressed{false};
	bool _booted{false};
	bool _booted_button_held{true};

	bool _below_idle_current{false};
	hrt_abstime _idle_start_time{0};

	bool _protections_enabled{true};

	bool _shutdown{false};

	char _manu_data[32]{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AUTO_PROTECT>)    _param_auto_protect,
		(ParamFloat<px4::params::PROTECT_CURRENT>)  _param_protect_current,
		(ParamInt<px4::params::IDLE_TIMEOUT>)    _param_idle_timeout,
		(ParamFloat<px4::params::IDLE_CURRENT>)    _param_idle_current,
		(ParamFloat<px4::params::PARALLEL_VOLTAGE>)    _param_parallel_voltage
	);
};
