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
#include <uORB/topics/watts_battery_status.h>

static constexpr uint16_t I2C_ADDR = 0x08;
static constexpr int BUS_NUMBER = 1;
static constexpr uint32_t FREQUENCY = 400000;

BQ76952::BQ76952() :
	I2C(DRV_DEVTYPE_BQ76952, "BQ76952", BUS_NUMBER, I2C_ADDR, FREQUENCY),
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

	configure_settings();
	px4_usleep(100_ms); // Give time for settings to take

	// Set FET mode to normal and configure FET actions when protections are tripped
	configure_fets();

	// After FET actions are configured, turn on most of the protections for logging, even though most FET actions are turned off
	enable_protections();

	// Enable LDO at REG1 if it's not already enabled (turns on the bq34)
	uint8_t value = 0b00001101;
	sub_command(CMD_REG12_CONTROL, &value, sizeof(value));
	px4_usleep(5_ms);

	return PX4_OK;
}

int BQ76952::configure_settings()
{
	enter_config_update_mode();

	int ret = PX4_OK;
	// ret |= write_register(Register<int16_t>{"Cell 1 Gain", 0x9180, 12125});
	// ret |= write_register(Register<int16_t>{"Cell 2 Gain", 0x9182, 12127});
	// ret |= write_register(Register<int16_t>{"Cell 3 Gain", 0x9184, 12125});
	// ret |= write_register(Register<int16_t>{"Cell 4 Gain", 0x9186, 12124});
	// ret |= write_register(Register<int16_t>{"Cell 5 Gain", 0x9188, 12125});
	// ret |= write_register(Register<int16_t>{"Cell 6 Gain", 0x918A, 12124});
	// ret |= write_register(Register<int16_t>{"Cell 7 Gain", 0x918C, 12125});
	// ret |= write_register(Register<int16_t>{"Cell 8 Gain", 0x918E, 12125});
	// ret |= write_register(Register<int16_t>{"Cell 9 Gain", 0x9190, 12123});
	// ret |= write_register(Register<int16_t>{"Cell 10 Gain", 0x9192, 12123});
	// ret |= write_register(Register<int16_t>{"Cell 11 Gain", 0x9194, 12127});
	// ret |= write_register(Register<int16_t>{"Cell 12 Gain", 0x9196, 12144});

	// ret |= write_register(Register<uint16_t>{"Pack Gain", 0x91A0, 35130});
	// ret |= write_register(Register<uint16_t>{"TOS Gain", 0x91A2, 33345});

	// ret |= write_register(Register<uint16_t>{"LD Gain", 0x91A4, 34500});

	//TODO - This is setting the wrong value. BQ76 expects type float4
	//ret |= write_register(Register<float>{"CC Gain", 0x91A8, 0.310f});

	//TODO - This is setting the wrong value. BQ76 expects type float4
	//ret |= write_register(Register<float>{"Capacity Gain", 0x91AC, 0.310f});

	ret |= write_register(Register<uint8_t>{"REG12 Config", 0x9236, 0x0d});
	ret |= write_register(Register<uint8_t>{"REG12 Config", 0x9237, 0x01});

	ret |= write_register(Register<uint8_t>{"TS1 Config", 0x92FD, 0x0f});

	ret |= write_register(Register<uint8_t>{"TS3 Config", 0x92FF, 0x07});

	ret |= write_register(Register<uint8_t>{"DA Configuration", 0x9303, 0x06});

	ret |= write_register(Register<uint16_t>{"Vcell Mode", 0x9304, 0x0fff});

	ret |= write_register(Register<uint16_t>{"Protection Configuration", 0x925F, 0x0000});

	// NOTE: we set these in the enable_protections() and disable_protections() functions
	// TODO: these don't match the settings I chose, please see the enable/disable functions
	{
		// "Settings","Protection","Protection Configuration","0000","Hex"
		// "Settings","Protection","Enabled Protections A","9c","Hex"
		// "Settings","Protection","Enabled Protections B","11","Hex"
		// "Settings","Protection","Enabled Protections C","40","Hex"
		// "Settings","Protection","CHG FET Protections A","18","Hex"
		// "Settings","Protection","CHG FET Protections B","11","Hex"
		// "Settings","Protection","CHG FET Protections C","10","Hex"
		// "Settings","Protection","DSG FET Protections A","84","Hex"
		// "Settings","Protection","DSG FET Protections B","00","Hex"
		// "Settings","Protection","DSG FET Protections C","40","Hex"
	}

	ret |= write_register(Register<int16_t>{"Precharge Start Voltage", 0x930A, 2800});
	ret |= write_register(Register<int16_t>{"Precharge Stop Voltage", 0x930C, 3000});

	ret |= write_register(Register<int16_t>{"Dsg Current Threshold", 0x9310, 35}); // What are userA units?
	ret |= write_register(Register<int16_t>{"Chg Current Threshold", 0x9312, 35}); // What are userA units?

	ret |= write_register(Register<uint16_t>{"Mfg Status Init", 0x9343, 0x0010}); // Enable FET normal mode

	ret |= write_register(Register<uint8_t>{"Balancing Configuration", 0x9335, 0x07});

	ret |= write_register(Register<uint8_t>{"Cell Balance Max Cells", 0x933A, 12});

	ret |= write_register(Register<int16_t>{"Sleep Current", 0x9248, 15});

	ret |= write_register(Register<uint8_t>{"COV Threshold", 0x9278, 84}); // units 50.6mV

	ret |= write_register(Register<uint8_t>{"COVL Latch Limit", 0x927D, 10});

	//TODO - This is getting set to 16, not 8
	//ret |= write_register(Register<uint8_t>{"OCC Threshold", 0x9280, 8});

	ret |= write_register(Register<uint8_t>{"SCD Threshold", 0x9286, 4}); // 4 = 80 mV across shunt. 80mv/400uOhm = 200A

	ret |= write_register(Register<uint8_t>{"SCD Delay", 0x9287, 31}); // units of 15us. 31 is max. 31 x 15us = 465us

	ret |= write_register(Register<uint8_t>{"SCDL Latch Limit", 0x9295, 10});

	exit_config_update_mode();

	return ret;
}

float BQ76952::temperature_cells()
{
	// Read external thermistor on TS3
	// TODO: sporadic erroneous values come from the BQ34 having TS enabled in the TEMPS bit mask in the Pack Configuration Register
	int16_t temperature = {};
	if (direct_command(CMD_READ_TS3_TEMP, &temperature, sizeof(temperature)) != PX4_OK) {
		PX4_ERR("Failed to read temperature");
		return 0;
	}
	return ((float)temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C
}

float BQ76952::temperature_fets()
{
	// Read board mounted resistor on TS1
	int16_t temperature = {};
	if (direct_command(CMD_READ_TS1_TEMP, &temperature, sizeof(temperature)) != PX4_OK) {
		PX4_ERR("Failed to read temperature");
		return 0;
	}
	return ((float)temperature / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Convert from 0.1K to C
}

float BQ76952::current()
{
	// TODO: fix units
	int16_t current = {};
	if (direct_command(CMD_READ_CC2_CURRENT, &current, sizeof(current)) != PX4_OK) {
		PX4_ERR("Failed to read current");
		return 0;
	}
	return (float)current / 100.0f; // centi-amp resolution 327amps +/-
}

float BQ76952::bat_voltage()
{
	int16_t stack_voltage = {};
	if (direct_command(CMD_READ_STACK_VOLTAGE, &stack_voltage, sizeof(stack_voltage)) != PX4_OK) {
		PX4_ERR("Failed to read bat voltage");
		return 0;
	}
	return (float)stack_voltage / 100.0f; // centi-volt
}

float BQ76952::pack_voltage()
{
	int16_t pack_voltage = {};
	if (direct_command(CMD_READ_PACK_PIN_VOLTAGE, &pack_voltage, sizeof(pack_voltage)) != PX4_OK) {
		PX4_ERR("Failed to read pack voltage");
		return 0;
	}
	return (float)pack_voltage / 100.0f; // centi-volt
}

void BQ76952::cell_voltages(float* cells_array, size_t size)
{
	int16_t cell_voltages_mv[12] = {};

	if (size > 12) size = 12;

	if (direct_command(CMD_READ_CELL_VOLTAGE, &cell_voltages_mv, size * sizeof(int16_t)) != PX4_OK) {
		PX4_ERR("Failed to read cell voltages");
		return;
	}

	for (size_t i = 0; i < size; i++) {
		cells_array[i] = (float)cell_voltages_mv[i] / 1000.0f;
	}
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

	configure_protections_fet_action();
}

uint16_t BQ76952::mfg_status()
{
	sub_command(CMD_MFG_STATUS);
	px4_usleep(5_ms);
	return sub_command_response16(0);
}

uint8_t BQ76952::otp_wr_check()
{
	sub_command(CMD_OTP_WR_CHECK);
	px4_usleep(10_ms);
	return sub_command_response8(0);
}

uint16_t BQ76952::battery_status()
{
	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
	return (buf[1] << 8) + buf[0];
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
			status_flags |= watts_battery_status_s::STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t under_volt_mask = (1 << 2);
		if (safety_status_a & under_volt_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_UNDER_VOLT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_volt_mask = (1 << 3);
		if (safety_status_a & over_volt_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_OVER_VOLT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_current_mask = (1 << 4) | (1 << 5) | (1 << 6);
		if (safety_status_a & over_current_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_OVER_CURRENT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t short_circuit_mask = (1 << 7);
		if (safety_status_a & short_circuit_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_SHORT_CIRCUIT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	// SAFETY ALERT/STATUS B
	{
		int ret = PX4_OK;
		uint8_t safety_status_b = {};
		ret |= direct_command(CMD_SAFETY_STATUS_B, &safety_status_b, sizeof(safety_status_b));


		if (ret != PX4_OK) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_temp_mask = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4);
		if (safety_status_b & over_temp_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_OVER_TEMP;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t under_temp_mask = (1 << 2) | (1 << 1) | (1 << 0);
		if (safety_status_b & under_temp_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_UNDER_TEMP;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	// SAFETY ALERT/STATUS C
	{
		int ret = PX4_OK;
		uint8_t safety_status_c = {};
		ret |= direct_command(CMD_SAFETY_STATUS_C, &safety_status_c, sizeof(safety_status_c));

		if (ret != PX4_OK) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_REQUIRES_SERVICE;
			return status_flags;
		}

		uint8_t over_current_mask = (1 << 7) | (1 << 5);
		if (safety_status_c & over_current_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_OVER_CURRENT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t short_circuit_mask = (1 << 6);
		if (safety_status_c & short_circuit_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_SHORT_CIRCUIT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}

		uint8_t over_volt_mask = (1 << 4);
		if (safety_status_c & over_volt_mask) {
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_OVER_VOLT;
			status_flags |= watts_battery_status_s::STATUS_FLAG_FAULT_PROTECTION_SYSTEM;
		}
	}

	return status_flags;
}

void BQ76952::enable_protections()
{
	PX4_INFO("Enabling protections");
	// SCDL_CURR_RECOV -- 1 = SCDL recovers when current is greater than or equal to Protections:SCDL:RecoveryTime.
	// OCDL_CURR_RECOV -- 1 = OCDL recovers when current is greater than or equal to Protections:OCDL:RecoveryTime
	// PF_OTP -- If this bit is not set, Permanent Failure status will be lost on any reset

	//  The individual protections can be enabled by setting the related Settings:Protection:Enabled Protections
	//  A – C configuration registers.

	enter_config_update_mode();

	// Settings:Protection:Enabled Protections A
	{
		uint8_t byte = {};

		byte |= 1 << 7; 	// 7 SCD 1 Short Circuit in Discharge Protection

		byte |= 1 << 6; 	// 6 OCD2 0 Overcurrent in Discharge 2nd Tier Protection

		byte |= 1 << 5; 	// 5 OCD1 0 Overcurrent in Discharge 1st Tier Protection
							// TODO: set the threshold

		byte |= 1 << 4; 	// 4 OCC 0 Overcurrent in Charge Protection

		byte |= 1 << 3; 	// 3 COV 1 Cell Overvoltage Protection

		byte |= 1 << 2; 	// 2 CUV 0 Cell Undervoltage Protection

		write_memory<uint8_t>(ADDR_PROTECTIONS_A, byte);
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

		write_memory<uint8_t>(ADDR_PROTECTIONS_B, byte);
	}

	// Settings:Protection:Enabled Protections C
	{
		uint8_t byte = {};

		byte |= 1 << 7;		// 7 OCD3 0 Overcurrent in Discharge 3rd Tier Protection

		byte |= 1 << 6; 	// 6 SCDL 0 Short Circuit in Discharge Latch
							// TODO: do we want to use this feature?

		byte |= 1 << 5; 	// 5 OCDL 0 Overcurrent in Discharge Latch
							// TODO: do we want to use this feature?

		byte |= 1 << 4; 	// 4 COVL 0 Cell Overvoltage Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 2; 	// 2 PTO 0 Precharge Timeout
							// TODO: do we want to use this feature?

		// byte |= 1 << 1; // 1 HWDF 0 Host Watchdog Fault

		write_memory<uint8_t>(ADDR_PROTECTIONS_C, byte);
	}

	exit_config_update_mode();
}

void BQ76952::disable_protections()
{
	PX4_INFO("Disabling protections");
	enter_config_update_mode();

	// Except these ones stay on
	{
		uint8_t byte = {};

		// byte |= 1 << 7; 	// 7 SCD 1 Short Circuit in Discharge Protection

		// byte |= 1 << 6; 	// 6 OCD2 0 Overcurrent in Discharge 2nd Tier Protection

		// byte |= 1 << 5; 	// 5 OCD1 0 Overcurrent in Discharge 1st Tier Protection
							// TODO: set the threshold

		byte |= 1 << 4; 	// 4 OCC 0 Overcurrent in Charge Protection

		byte |= 1 << 3; 	// 3 COV 1 Cell Overvoltage Protection

		// byte |= 1 << 2; 	// 2 CUV 0 Cell Undervoltage Protection

		write_memory<uint8_t>(ADDR_PROTECTIONS_A, byte);
	}
	{
		uint8_t byte = {};

		// byte |= 1 << 7; 	// 7 OTF 0 FET Overtemperature

		// byte |= 1 << 6; 	// 6 OTINT 0 Internal Overtemperature

		// byte |= 1 << 5; 	// 5 OTD 0 Overtemperature in Discharge

		byte |= 1 << 4; 	// 4 OTC 0 Overtemperature in Charge

		// byte |= 1 << 2; 	// 2 UTINT 0 Internal Undertemperature

		// byte |= 1 << 1; 	// 1 UTD 0 Undertemperature in Discharge

		byte |= 1 << 0; 	// 0 UTC 0 Undertemperature in Charge

		write_memory<uint8_t>(ADDR_PROTECTIONS_B, byte);
	}
	{
		uint8_t byte = {};

		// byte |= 1 << 7; // 7 OCD3 0 Overcurrent in Discharge 3rd Tier Protection

		// byte |= 1 << 6; 	// 6 SCDL 0 Short Circuit in Discharge Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 5; 	// 5 OCDL 0 Overcurrent in Discharge Latch
							// TODO: do we want to use this feature?

		byte |= 1 << 4; 	// 4 COVL 0 Cell Overvoltage Latch
							// TODO: do we want to use this feature?

		// byte |= 1 << 2; 	// 2 PTO 0 Precharge Timeout
							// TODO: do we want to use this feature?

		// byte |= 1 << 1; // 1 HWDF 0 Host Watchdog Fault

		write_memory<uint8_t>(ADDR_PROTECTIONS_C, byte);
	}

	exit_config_update_mode();
}

void BQ76952::configure_protections_fet_action()
{
	PX4_INFO("Configuring FET protection actions");

	enter_config_update_mode();

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

		write_memory<uint8_t>(ADDR_CHG_FET_Protections_A, byte);
	}

	// CHG FET Protections B
	{
		uint8_t byte = {};

		// 7 OTF 1 FET Overtemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 7);
		// TODO: set threshold

		// 6 OTINT 1 Internal Overtemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 6);
		// TODO: set threshold

		// 4 OTC 1 Overtemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);
		// TODO: set threshold

		// 2 UTINT 1 Internal Undertemperature
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 2);
		// TODO: set threshold

		// 0 UTC 1 Undertemperature in Charge
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 0);
		// TODO: set threshold

		write_memory<uint8_t>(ADDR_CHG_FET_Protections_B, byte);
	}

	// CHG FET Protections C
	{
		uint8_t byte = {};

		// 6 SCDL 1 Short Circuit in Discharge Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 6);

		// 4 COVL 1 Cell Overvoltage Latch
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		byte |= (1 << 4);

		// 2 PTO 1 Precharge Timeout
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 2);

		// 1 HWDF 1 Host Watchdog Fault
		//      0 = CHG FET is not disabled when protection is triggered.
		//      1 = CHG FET is disabled when protection is triggered.
		// byte |= (1 << 0);

		// TODO: do we want to use these features? ^

		write_memory<uint8_t>(ADDR_CHG_FET_Protections_C, byte);
	}

	// DSG FET Protections A
	{
		uint8_t byte = {};

		// 7 SCD 1 Short Circuit in Discharge Protection
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 7);

		// 6 OCD2 1 Overcurrent in Discharge 2nd Tier Protection
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 6);

		// 5 OCD1 1 Overcurrent in Discharge 1st Tier Protection
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 5);

		// 2 CUV 1 Cell Undervoltage Protection
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		byte |= (1 << 2);

		write_memory<uint8_t>(ADDR_DSG_FET_Protections_A, byte);
	}

	// DSG FET Protections B
	{
		uint8_t byte = {};

		// 7 OTF 1 FET Overtemperature
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 7);

		// 6 OTINT 1 Internal Overtemperature
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 6);

		// 5 OTD 1 Overtemperature in Discharge
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 5);

		// 2 UTINT 1 Internal Undertemperature
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 2);

		// 1 UTD 1 Undertemperature in Discharge
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 1);

		write_memory<uint8_t>(ADDR_DSG_FET_Protections_B, byte);
	}

	// DSG FET Protections C
	{
		uint8_t byte = {};
		// 7 OCD3 1 Overcurrent in Discharge 3rd Tier Protection
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 7);

		// 6 SCDL 1 Short Circuit in Discharge Latch
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 6);

		// 5 OCDL 1 Overcurrent in Discharge Latch
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 5);

		// 1 HWDF 1 Host Watchdog Fault
		// 		0 = DSG FET is not disabled when protection is triggered.
		// 		1 = DSG FET is disabled when protection is triggered.
		// byte |= (1 << 1);

		write_memory<uint8_t>(ADDR_DSG_FET_Protections_C, byte);
	}

	exit_config_update_mode();
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

int BQ76952::enter_config_update_mode()
{
	// PX4_INFO("Entering CONFIG_UPDATE mode");
	// Enter config udpate mode if not already in it and report status
	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));

	// TODO: w/ new board and battery the firs time we enter CONFIG_UPDATE it fails, why???
	// We will compensate by trying up to 3 times
	int retries = 0;
	while (retries < 3) {

		px4_usleep(5_ms);
		if (!(buf[0] & 0x01)) {
			sub_command(CMD_SET_CFGUPDATE);
			px4_usleep(10_ms); // 2000us time to complete operation
			direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
			px4_usleep(5_ms);
			if (buf[0] & 0x01) {
				// PX4_INFO("Successfully went into config update mode");
				return PX4_OK;
			}
		} else {
			// Already in config update mode
			PX4_INFO("Already in config update mode");
			return PX4_OK;
		}

		PX4_INFO("FAILED to enter CONFIG_UPDATE mode");
		retries++;
	}

	return PX4_ERROR;
}

int BQ76952::exit_config_update_mode()
{
	// PX4_INFO("Exiting CONFIG_UPDATE mode");

	sub_command(CMD_EXIT_CFG_UPDATE);
	px4_usleep(10_ms); // 1000us time to complete operation

	uint8_t buf[2] = {};
	direct_command(CMD_BATTERY_STATUS, buf, sizeof(buf));
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
	px4_usleep(1_ms);
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

int BQ76952::probe()
{
	// Check device number matches what we expect
	int ret = sub_command(CMD_DEVICE_NUMBER);
	px4_usleep(5_ms);
	uint16_t number = sub_command_response16(0);
	PX4_INFO("device number: 0x%x", number);

	if (number != 0x7695) {
		return PX4_ERROR;
	}

	return ret;
}
