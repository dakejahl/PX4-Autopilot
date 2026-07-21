/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

// NOTE: this part is functionality equivalent to the lis2mdl
// https://www.st.com/resource/en/design_tip/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics.pdf
// https://www.st.com/resource/en/datasheet/iis2mdc.pdf

#include "iis2mdc.h"

#include <string.h>

using namespace time_literals;

// CFG_REG_B filter configurations selected by IIS2MDC_FILT. Offset cancellation
// averages a set/reset measurement pair, which also halves the bandwidth, so the
// LPF-only entry exists to separate the two effects when comparing.
static constexpr uint8_t FILTER_CONFIG[] = {
	0,        // 0: neither
	OFF_CANC, // 1: offset cancellation, the AN5080 reference configuration
	LPF,      // 2: LPF only, bandwidth matched to offset cancellation
};
static constexpr int FILTER_CONFIG_COUNT = sizeof(FILTER_CONFIG) / sizeof(FILTER_CONFIG[0]);
static constexpr int FILTER_MODE_CYCLE = FILTER_CONFIG_COUNT;

IIS2MDC::IIS2MDC(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_interface(interface),
	_px4_mag(interface->get_device_id(), config.rotation),
	_sample_count(perf_alloc(PC_COUNT, "iis2mdc_read")),
	_comms_errors(perf_alloc(PC_COUNT, "iis2mdc_comms_errors")),
	_data_not_ready(perf_alloc(PC_COUNT, "iis2mdc_not_ready"))
{}

IIS2MDC::~IIS2MDC()
{
	perf_free(_sample_count);
	perf_free(_comms_errors);
	perf_free(_data_not_ready);
	delete _interface;
}

int IIS2MDC::init()
{
	if (hrt_absolute_time() < 20_ms) {
		px4_usleep(20_ms); // ~10ms power-on time
	}

	write_register(IIS2MDC_ADDR_CFG_REG_A, MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN);
	write_register(IIS2MDC_ADDR_CFG_REG_C, BDU);

	ModuleParams::updateParams();
	ParametersUpdate(true); // writes CFG_REG_B

	_px4_mag.set_scale(0.0015f); // 1.5 mGauss/LSB

	ScheduleDelayed(20_ms);

	return PX4_OK;
}

void IIS2MDC::ParametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();
	const int32_t mode = _param_iis2mdc_filt.get();
	uint8_t cfg_reg_b = OFF_CANC;

	if (mode == FILTER_MODE_CYCLE) {
		int32_t cycle_ms = _param_iis2mdc_cycle.get();

		if (cycle_ms < 100) {
			cycle_ms = 100;
		}

		if (_cycle_last == 0) {
			_cycle_last = now;
		}

		if (now - _cycle_last >= (hrt_abstime)cycle_ms * 1000) {
			_cycle_last = now;
			_cycle_index = (_cycle_index + 1) % FILTER_CONFIG_COUNT;
		}

		cfg_reg_b = FILTER_CONFIG[_cycle_index];

	} else {
		int index = mode;

		if (index < 0) {
			index = 0;

		} else if (index >= FILTER_CONFIG_COUNT) {
			index = FILTER_CONFIG_COUNT - 1;
		}

		_cycle_last = 0;
		_cycle_index = 0;
		cfg_reg_b = FILTER_CONFIG[index];
	}

	if (cfg_reg_b != _cfg_reg_b || force) {
		_cfg_reg_b = cfg_reg_b;
		write_register(IIS2MDC_ADDR_CFG_REG_B, _cfg_reg_b);
		PublishConfig(now);

	} else if (now - _config_published >= 1_s) {
		PublishConfig(now);
	}
}

void IIS2MDC::PublishConfig(const hrt_abstime &now)
{
	debug_key_value_s dbg{};
	strncpy(dbg.key, "mdc_cfgb", sizeof(dbg.key));
	dbg.value = (float)_cfg_reg_b;
	dbg.timestamp = now;
	_debug_pub.publish(dbg);

	_config_published = now;
}

void IIS2MDC::RunImpl()
{
	ParametersUpdate();

	uint8_t status = read_register(IIS2MDC_ADDR_STATUS_REG);

	if (status & IIS2MDC_STATUS_REG_READY) {
		SensorData data = {};

		if (read_register_block(&data) == PX4_OK) {
			int16_t x = int16_t((data.xout1 << 8) | data.xout0);
			int16_t y = int16_t((data.yout1 << 8) | data.yout0);
			int16_t z = -int16_t((data.zout1 << 8) | data.zout0);
			int16_t t = int16_t((data.tout1 << 8) | data.tout0);
			// 16 bits twos complement with a sensitivity of 8 LSB/°C. Typically, the output zero level corresponds to 25 °C.
			_px4_mag.set_temperature(float(t) / 8.f + 25.f);
			_px4_mag.update(hrt_absolute_time(), x, y, z);
			_px4_mag.set_error_count(perf_event_count(_comms_errors));
			perf_count(_sample_count);

		} else {
			PX4_DEBUG("read failed");
			perf_count(_comms_errors);
		}

	} else {
		// Polled at twice the ODR so no sample is missed to clock drift; roughly
		// half the polls find no new data, which is not an error.
		perf_count(_data_not_ready);
	}

	ScheduleDelayed(5_ms);
}

uint8_t IIS2MDC::read_register_block(SensorData *data)
{
	uint8_t reg = IIS2MDC_ADDR_OUTX_L_REG;

	if (_interface->read(reg, data, sizeof(SensorData)) != PX4_OK) {
		perf_count(_comms_errors);

		return PX4_ERROR;
	}

	return PX4_OK;
}

uint8_t IIS2MDC::read_register(uint8_t reg)
{
	uint8_t value = 0;

	if (_interface->read(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}

	return value;
}

void IIS2MDC::write_register(uint8_t reg, uint8_t value)
{
	if (_interface->write(reg, &value, sizeof(value)) != PX4_OK) {
		perf_count(_comms_errors);
	}
}

void IIS2MDC::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("CFG_REG_B: 0x%02x (OFF_CANC %s, LPF %s)", _cfg_reg_b,
		 (_cfg_reg_b & OFF_CANC) ? "on" : "off",
		 (_cfg_reg_b & LPF) ? "on" : "off");
	perf_print_counter(_sample_count);
	perf_print_counter(_comms_errors);
	perf_print_counter(_data_not_ready);
}
