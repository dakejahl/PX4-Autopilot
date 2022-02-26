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
	direct_command(CMD_READ_VOLTAGE_STACK, buf, sizeof(buf));

	float stack_voltage = (buf[1] << 8) | buf[0];
	PX4_INFO("stack_voltage: %f", double(stack_voltage / 100)); // units of 0.01v

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
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	// Any other initialization we need to do?

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

int WattsBms::direct_command(uint8_t command, uint8_t* buf, size_t len)
{
	return transfer(&command, 1, buf, len);
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
