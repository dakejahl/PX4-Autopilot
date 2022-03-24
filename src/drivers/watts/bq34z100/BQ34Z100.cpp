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

// The bq34z100-G1 device is an Impedance Track™ fuel gauge for Li-Ion, PbA, NiMH, and NiCd batteries,
// and works independently of battery series-cell configurations

// FUNCTION OF THE BQ34Z100
// Read current consumed
// Read energy consumed
// Read percent remaining
// Read time remaining
// Read mAh consumed

#include "BQ34Z100.hpp"

BQ34Z100::BQ34Z100(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
	_battery_status_pub.advertise();
}


BQ34Z100::~BQ34Z100()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	_battery_status_pub.unadvertise();
}

void BQ34Z100::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void BQ34Z100::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("exiting");
		return;
	}

	perf_begin(_cycle_perf);

	// Collect data and publish on uORB --> UAVCAN
	{
		battery_status_s battery_status = {};
		battery_status.timestamp = hrt_absolute_time();

		int ret = PX4_OK;

		// TODO: read data
		// float voltage = read_voltage();
		// PX4_INFO("voltage: %fv", double(voltage));
		// uint8_t soc = read_soc();
		// PX4_INFO("soc: %u%%", soc);

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
		}

		// Publish to uORB
		_battery_status_pub.publish(battery_status);
	}

	perf_end(_cycle_perf);
}

void BQ34Z100::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bq34z100", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void BQ34Z100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

int BQ34Z100::probe()
{
	static constexpr uint16_t DEVICE_TYPE_EXPECTED = 0x0100;
	uint8_t val = {};

	int ret = transfer(&val, sizeof(val), nullptr, 0);
	if (ret == PX4_OK) {
		if (read_device_type() == DEVICE_TYPE_EXPECTED) {
			return PX4_OK;
		}
	}

	return -1;
}

int BQ34Z100::init()
{
	PX4_INFO("Initializing BQ34Z100");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	///// WRITE SETTINGS INTO PERMANENT MEMORY /////
	// TODO?

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

uint8_t BQ34Z100::read_soc()
{
	static constexpr uint8_t REG_ADDR_SOC = 0x02;
	return read_register8(REG_ADDR_SOC);
}

float BQ34Z100::read_voltage()
{
	static constexpr uint8_t REG_ADDR_VOLTAGE = 0x08;
	return float(read_register16(REG_ADDR_VOLTAGE)) / 1000.0f;
}

uint16_t BQ34Z100::read_device_type()
{
	return read_control(0x00, 0x01);
}

uint16_t BQ34Z100::read_control(uint8_t addr_msb, uint8_t addr_lsb)
{
	static constexpr uint8_t CONTROL_REG = 0x00;
	uint8_t command[] = {CONTROL_REG, addr_lsb, addr_msb};
	int ret = transfer(command, sizeof(command), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("read_control addr 0x%x%x failed", addr_msb, addr_lsb);
		return 0;
	}

	return read_register16(CONTROL_REG);
}

uint8_t BQ34Z100::read_register8(uint8_t addr)
{
	int ret = transfer(&addr, sizeof(addr), nullptr, 0);
	uint8_t recv = {};
	ret |= transfer(nullptr, 0, &recv, sizeof(recv));

	if (ret != PX4_OK) {
		PX4_ERR("read_register8 addr 0x%x failed", addr);
		return 0;
	}

	return recv;
}

uint16_t BQ34Z100::read_register16(uint8_t addr)
{
	int ret = transfer(&addr, 1, nullptr, 0);
	uint8_t recv[2] = {};
	ret |= transfer(nullptr, 0, recv, sizeof(recv));

	if (ret != PX4_OK) {
		PX4_ERR("read_register16 addr 0x%x failed", addr);
		return 0;
	}

	uint16_t value = (recv[1] << 8) | recv[0];
	return value;
}
