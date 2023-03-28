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

#include "BQ34Z100.hpp"

static constexpr uint16_t I2C_ADDR = 0x55;
static constexpr int BUS_NUMBER = 1;
static constexpr uint32_t FREQUENCY = 400000;

BQ34Z100::BQ34Z100() :
	I2C(DRV_DEVTYPE_BQ34Z100, "BQ34Z100", BUS_NUMBER, I2C_ADDR, FREQUENCY),
	_comms_errors(perf_alloc(PC_COUNT, "BQ34Z100: comm errors"))
{}

BQ34Z100::~BQ34Z100()
{
	perf_free(_comms_errors);
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

	PX4_ERR("device type does not match expected!");
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

	return PX4_OK;
}

// DataFlashBlock(): 0x3F
// Issuing a 0x01 instructs the BlockData() command to transfer Manufacturer Data

// BlockDataControl(): 0x61
// UNSEALED Access: This command is used to control data flash ACCESS mode. Writing 0x00 to this
// command enables BlockData() to access general data flash. Writing a 0x01 to this command enables the
// SEALED mode operation of DataFlashBlock().

// BlockDataChecksum(): 0x60
// UNSEALED Access: This byte contains the checksum on the 32 bytes of block data read or written to
// data flash.
// SEALED Access: This byte contains the checksum for the 32 bytes of block data written to Manufacturer
// Data.

// To access data flash locations individually, the block containing the desired data flash location(s) must be
// transferred to the command register locations where they can be read to the host or changed directly. This
// is accomplished by sending the set-up command BlockDataControl() (code 0x61) with data 0x00. Up to 32
// bytes of data can be read directly from the BlockData() command locations 0x40…0x5F, externally
// altered, then re-written to the BlockData() command space. Alternatively, specific locations can be read,
// altered, and re-written if their corresponding offsets are used to index into the BlockData() command
// space. Finally, the data residing in the command space is transferred to data flash, once the correct
// checksum for the whole block is written to BlockDataChecksum() (command number 0x60).

float BQ34Z100::read_voltage()
{
	static constexpr uint8_t REG_ADDR_VOLTAGE = 0x08;
	return float(read_register<uint16_t>(REG_ADDR_VOLTAGE)) / 1000.0f;
}

// State of charge 0 - 100
uint32_t BQ34Z100::read_state_of_charge()
{
	static constexpr uint8_t READ_ADDR_STATE_OF_CHARGE = 0x02;
	return uint32_t(read_register<uint16_t>(READ_ADDR_STATE_OF_CHARGE));
}

// Remaining capacity mAh
uint32_t BQ34Z100::read_remaining_capacity()
{
	static constexpr uint8_t REG_ADDR_REMAINING_CAPACITY = 0x04;
	return uint32_t(read_register<uint16_t>(REG_ADDR_REMAINING_CAPACITY));
}

// Full charge capacity mAh. However, if PackConfiguration [SCALED] is set then the units have been scaled
// through the calibration process. The actual scale is not set in the device and SCALED is just an indicator flag.
// The calibration scale is set through the parameter CAPACITY_SCALAR.
uint32_t BQ34Z100::read_full_charge_capacity()
{
	static constexpr uint8_t REG_ADDR_FULL_CHARGE_CAPACITY = 0x06;
	return uint32_t(read_register<uint16_t>(REG_ADDR_FULL_CHARGE_CAPACITY));
}

uint32_t BQ34Z100::read_design_capacity()
{
	static constexpr uint8_t REG_ADDR_DESIGN_CAPACITY = 0x3C;
	return uint32_t(read_register<uint16_t>(REG_ADDR_DESIGN_CAPACITY));
}

uint16_t BQ34Z100::read_cycle_count()
{
	static constexpr uint8_t REG_ADDR_CYCLE_COUNT = 0x2C;
	return uint16_t(read_register<uint16_t>(REG_ADDR_CYCLE_COUNT));
}

uint16_t BQ34Z100::read_serial_number()
{
	static constexpr uint8_t REG_ADDR_SERIAL_NUMBER = 0x2E;
	return uint16_t(read_register<uint16_t>(REG_ADDR_SERIAL_NUMBER));
}

uint8_t BQ34Z100::read_state_of_health()
{
	static constexpr uint8_t REG_ADDR_CYCLE_COUNT = 0x2E;
	return uint8_t(read_register<uint16_t>(REG_ADDR_CYCLE_COUNT));
}

uint16_t BQ34Z100::read_device_type()
{
	return read_control(0x00, 0x01);
}

uint16_t BQ34Z100::read_control_status()
{
	return read_control(0x00, 0x00);
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

	return read_register<uint16_t>(CONTROL_REG);
}
