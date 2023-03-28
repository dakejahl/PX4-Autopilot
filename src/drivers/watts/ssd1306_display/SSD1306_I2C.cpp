/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "SSD1306_I2C.h"

static constexpr int BUS_NUMBER = 2;
static constexpr uint32_t FREQUENCY = 400000;

SSD1306_I2C::SSD1306_I2C(uint16_t address) :
	I2C(DRV_DEVTYPE_SSD1306, "SSD1306_I2C", BUS_NUMBER, address, FREQUENCY),
	_comms_errors(perf_alloc(PC_COUNT, "SSD1306_I2C: comm errors"))
{
	setGeometry(OLEDDISPLAY_GEOMETRY::GEOMETRY_128_64);

	_buffer2 = new uint8_t[_displayBufferSize + 1];

	PX4_INFO("Initializing SSD1306_I2C");
	I2C::init();
}

bool SSD1306_I2C::connect()
{
	return true;
}

void SSD1306_I2C::display()
{
	const int x_offset = (128 - this->width()) / 2;

	sendCommand(COLUMNADDR);
	sendCommand(x_offset);						// column start address (0 = reset)
	sendCommand(x_offset + (this->width() - 1));// column end address (127 = reset)

	sendCommand(PAGEADDR);
	sendCommand(0x0);							// page start address (0 = reset)

	if (geometry == GEOMETRY_128_64) {
		sendCommand(0x7);
	} else if (geometry == GEOMETRY_128_32) {
		sendCommand(0x3);
	}

	_buffer2[0] = 0x40;
	memcpy(_buffer2 + 1, _buffer, _displayBufferSize);
	int ret = transfer(_buffer2, _displayBufferSize + 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}
}

void SSD1306_I2C::sendCommand(uint8_t command)
{
	uint8_t data[2] = {0x80, command}; // control + command

	int ret = transfer((uint8_t*)&data, sizeof(data), nullptr, 0);
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}
}
