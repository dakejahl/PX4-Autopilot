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
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>
#include <uORB/Publication.hpp>

using namespace time_literals;

class BQ34Z100 : public device::I2C, public I2CSPIDriver<BQ34Z100>
{
public:
	BQ34Z100(const I2CSPIDriverConfig &config);
	~BQ34Z100() override;

	int init() override;

	static void print_usage();

	void RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:
    // This repo is quite good
    // https://github.com/xkam1x/BQ34Z100G1/blob/master/bq34z100g1.cpp

    int probe() override;

    uint8_t read_soc();
    float read_voltage();
    uint16_t read_device_type();

    uint16_t read_control(uint8_t addr_msb, uint8_t addr_lsb);
    uint8_t read_register8(uint8_t addr);
    uint16_t read_register16(uint8_t addr);

private:
    static const hrt_abstime    SAMPLE_INTERVAL {500_ms};

	uORB::Publication<battery_status_s>	_battery_status_pub {ORB_ID(battery_status)};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;
};
