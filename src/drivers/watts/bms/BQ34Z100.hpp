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

using namespace time_literals;

class BQ34Z100 : public device::I2C
{
public:
	BQ34Z100();
	~BQ34Z100();

    // Initialize things
	int init();
    int probe() override;

    // Monitor things
    float read_voltage();
    uint32_t read_state_of_charge();
    uint32_t read_remaining_capacity();
    uint32_t read_full_charge_capacity();
    uint32_t read_design_capacity();
    uint16_t read_cycle_count();
    uint16_t read_serial_number();
    uint8_t read_state_of_health();

    uint16_t read_device_type();
    uint16_t read_control_status();

    // Memory access
    uint16_t read_control(uint8_t addr_msb, uint8_t addr_lsb);

    template <typename T>
    T read_register(uint8_t addr)
    {
        T data = {};
        int ret = transfer(&addr, 1, nullptr, 0);
        ret |= transfer(nullptr, 0, (uint8_t*)&data, sizeof(data));
        px4_usleep(5_ms);

        if (ret != PX4_OK) {
            PX4_ERR("read_register addr 0x%x failed", addr);
            return 0;
        }

        return data;
    }

private:
	perf_counter_t _comms_errors;
};
