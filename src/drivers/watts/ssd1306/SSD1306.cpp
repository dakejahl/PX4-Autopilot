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

#include "SSD1306.hpp"

using namespace time_literals;

SSD1306::SSD1306(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{}

SSD1306::~SSD1306()
{
	ScheduleClear();

	if (_buffer) {
		delete[] _buffer;
	}

	perf_free(_cycle_perf);
}

void SSD1306::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

int SSD1306::init()
{
	PX4_INFO("Initializing SSD1306");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}
	if (!_buffer) {
		_buffer = new uint8_t[DISPLAY_BUFFER_SIZE];
		_buffer2 = new uint8_t[DISPLAY_BUFFER_SIZE + 1];

		if( !_buffer) {
			PX4_INFO("Not enough memory to create display");
			return PX4_ERROR;
		}
	}

	usleep(10000);
	sendInitCommands();
	resetDisplay();

	usleep(10000);

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);

	return PX4_OK;
}

void SSD1306::RunImpl()
{
	perf_begin(_cycle_perf);

	battery_status_s battery;

	if (_battery_sub.update(&battery)) {
		updateStatus(battery);
	}

	// TODO: implement a mechanism to safely shut down -- SEE button_task.cpp
	// shutdown_s shutdown;
	// if (_shutdown_sub.update(&shutdown)) {
	// 	PX4_INFO("SHUTTING DOWN NOW");
	// 	displayOff();
	// 	exit_and_cleanup();
	// }

	perf_end(_cycle_perf);
}

void SSD1306::updateStatus(const battery_status_s& data)
{
	clear();

	char text_temp[20] = {};
	const char* str;

	snprintf(text_temp, sizeof(text_temp), "mV: %d", (int)(data.voltage_v*1000));
	str = text_temp;
	drawString(0, 0, str);

	// TODO: verify we are properly handling negatives here.
	snprintf(text_temp, sizeof(text_temp), "mA: %d", (int)(data.current_a*-1000));

	str = text_temp;
	drawString(0, 16, str);

	snprintf(text_temp, sizeof(text_temp), "%d%%", (int)(data.remaining*100));
	str = text_temp;
	drawString(85, 0, str);

	display();
}

void SSD1306::display(void)
{
	sendCommand(COLUMNADDR);
	sendCommand(0x0);
	sendCommand(0x7F);

	sendCommand(PAGEADDR);
	sendCommand(0x0);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0x7);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x3);
	}

	sendData(_buffer, DISPLAY_BUFFER_SIZE);
}

void SSD1306::sendInitCommands(void)
{
	sendCommand(DISPLAYOFF);
	sendCommand(SETDISPLAYCLOCKDIV);
	sendCommand(0xF0); // Increase speed of the display max ~96Hz
	sendCommand(SETMULTIPLEX);
	sendCommand(DISPLAY_HEIGHT - 1);
	sendCommand(SETDISPLAYOFFSET);
	sendCommand(0x00);
	sendCommand(SETSTARTLINE);
	sendCommand(CHARGEPUMP);
	sendCommand(0x14);
	sendCommand(MEMORYMODE);
	sendCommand(0x00);
	sendCommand(SEGREMAP);
	sendCommand(COMSCANINC);
	sendCommand(SETCOMPINS);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0x12);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x02);
	}

	sendCommand(SETCONTRAST);

	if (_geometry == GEOMETRY_128_64)
	{
		sendCommand(0xCF);
	}
	else if (_geometry == GEOMETRY_128_32)
	{
		sendCommand(0x8F);
	}

	sendCommand(SETPRECHARGE);
	sendCommand(0xF1);
	sendCommand(SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
	sendCommand(0x40);          //0x40 default, to lower the contrast, put 0
	sendCommand(DISPLAYALLON_RESUME);
	sendCommand(NORMALDISPLAY);
	sendCommand(0x2e);            // stop scroll
	sendCommand(DISPLAYON);
}

void SSD1306::drawString(int16_t xMove, int16_t yMove, const char* text)
{
	uint16_t lineHeight = pgm_read_byte(_fontData + HEIGHT_POS);

	uint16_t yOffset = 0;
	// If the string should be centered vertically too then we need to know how heigh the string is.
	if (_textAlignment == TEXT_ALIGN_CENTER_BOTH)
	{
		uint16_t lb = 0;
		// Find number of linebreaks in text
		for (uint16_t i=0;text[i] != 0; i++)
		{
			lb += (text[i] == 10);
		}

		// Calculate center
		yOffset = (lb * lineHeight) / 2;
	}

	uint16_t length = strlen(text);
	drawStringInternal(xMove, yMove - yOffset, text, length, getStringWidth(text, length));
}

void SSD1306::drawStringInternal(int16_t xMove, int16_t yMove, const char* text, uint16_t textLength, uint16_t textWidth)
{
	uint8_t textHeight       = pgm_read_byte(_fontData + HEIGHT_POS);
	uint8_t firstChar        = pgm_read_byte(_fontData + FIRST_CHAR_POS);
	uint16_t sizeOfJumpTable = pgm_read_byte(_fontData + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

	uint16_t cursorX         = 0;
	uint16_t cursorY         = 0;

	switch (_textAlignment)
	{
		case TEXT_ALIGN_CENTER_BOTH:
			yMove -= textHeight >> 1;
		// Fallthrough
		case TEXT_ALIGN_CENTER:
			xMove -= textWidth >> 1; // divide by 2
			break;
		case TEXT_ALIGN_RIGHT:
			xMove -= textWidth;
			break;
		case TEXT_ALIGN_LEFT:
			break;
	}

	// Don't draw anything if it is not on the screen.
	if (xMove + textWidth  < 0 || xMove > DISPLAY_WIDTH )
	{
		return;
	}

	if (yMove + textHeight < 0 || yMove > DISPLAY_WIDTH )
	{
		return;
	}

	for (uint16_t j = 0; j < textLength; j++)
	{
		int16_t xPos = xMove + cursorX;
		int16_t yPos = yMove + cursorY;

		uint8_t code = text[j];
		if (code >= firstChar)
		{
			uint8_t charCode = code - firstChar;

			// 4 Bytes per char code
			uint8_t msbJumpToChar    = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
			uint8_t lsbJumpToChar    = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
			uint8_t charByteSize     = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
			uint8_t currentCharWidth = pgm_read_byte( _fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

			// Test if the char is drawable
			if (!(msbJumpToChar == 255 && lsbJumpToChar == 255))
			{
				// Get the position of the char data
				uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
				drawInternal(xPos, yPos, currentCharWidth, textHeight, _fontData, charDataPosition, charByteSize);
			}

			cursorX += currentCharWidth;
		}
	}
}

void SSD1306::drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData)
{
	if (width < 0 || height < 0)
	{
		return;
	}
	if (yMove + height < 0 || yMove > DISPLAY_HEIGHT)
	{
		return;
	}
	if (xMove + width  < 0 || xMove > DISPLAY_WIDTH)
	{
		return;
	}

	uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
	int8_t   yOffset      = yMove & 7;

	bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

	int16_t initYMove   = yMove;
	int8_t  initYOffset = yOffset;


	for (uint16_t i = 0; i < bytesInData; i++)
	{
		// Reset if next horizontal drawing phase is started.
		if ( i % rasterHeight == 0)
		{
			yMove   = initYMove;
			yOffset = initYOffset;
		}

		uint8_t currentByte = pgm_read_byte(data + offset + i);

		int16_t xPos = xMove + (i / rasterHeight);
		int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * DISPLAY_WIDTH;

		int16_t dataPos    = xPos  + yPos;

		if (dataPos >=  0  && dataPos < DISPLAY_BUFFER_SIZE &&
			xPos    >=  0  && xPos    < DISPLAY_WIDTH )
		{

			if (yOffset >= 0)
			{
				_buffer[dataPos] |= currentByte << yOffset;

				if (dataPos < (DISPLAY_BUFFER_SIZE - DISPLAY_WIDTH))
				{
					_buffer[dataPos + DISPLAY_WIDTH] |= currentByte >> (8 - yOffset);
				}
			}
			else
			{
				// Make new offset position
				yOffset = -yOffset;

				_buffer[dataPos] |= currentByte >> yOffset;

				// Prepare for next iteration by moving one block up
				yMove -= 8;

				// and setting the new yOffset
				yOffset = 8 - yOffset;
			}
		}
	}
}

uint16_t SSD1306::getStringWidth(const char* text, uint16_t length)
{
	uint16_t firstChar = pgm_read_byte(_fontData + FIRST_CHAR_POS);
	uint16_t stringWidth = 0;
	uint16_t maxWidth = 0;

	while (length--)
	{
		stringWidth += pgm_read_byte(_fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
		if (text[length] == 10)
		{
			maxWidth = math::max(maxWidth, stringWidth);
			stringWidth = 0;
		}
	}

	return math::max(maxWidth, stringWidth);
}

void SSD1306::sendCommand(uint8_t command)
{
	uint8_t tx[2] = {0x80, command};
	writeBytes(tx, 2);
}

void SSD1306::sendData(uint8_t* data, size_t size)
{
	_buffer2[0] = 0x40;
	memcpy(_buffer2 + 1, data, size);
	writeBytes(_buffer2, size + 1);
}

void SSD1306::resetDisplay(void)
{
	clear();
	display();
	flipScreenVertically(); // We've got to flip it for it to be correct for our use
}

void SSD1306::flipScreenVertically()
{
	sendCommand(SEGREMAP | 0x01);
	sendCommand(COMSCANDEC); // Rotate screen 180 Deg
}

void SSD1306::displayOn(void)
{
	sendCommand(DISPLAYON);
}

void SSD1306::displayOff(void)
{
	sendCommand(DISPLAYOFF);
}

void SSD1306::clear(void)
{
	memset(_buffer, 0, DISPLAY_BUFFER_SIZE);
}

void SSD1306::setFontTableLookupFunction(FontTableLookupFunction function)
{
	_fontTableLookupFunction = function;
}

char DefaultFontTableLookup(const uint8_t ch)
{
	// UTF-8 to font table index converter
	// Code form http://playground.arduino.cc/Main/Utf8ascii
	static uint8_t LASTCHAR;

	if (ch < 128)
	{ // Standard ASCII-set 0..0x7F handling
		LASTCHAR = 0;
		return ch;
	}

	uint8_t last = LASTCHAR;   // get last char
	LASTCHAR = ch;

	switch (last)
	{    // conversion depnding on first UTF8-character
		case 0xC2: return (uint8_t) ch;
		case 0xC3: return (uint8_t) (ch | 0xC0);
		case 0x82: if (ch == 0xAC) return (uint8_t) 0x80;    // special case Euro-symbol
	}

	return (uint8_t) 0; // otherwise: return zero, if character has to be ignored
}

void SSD1306::writeBytes(uint8_t* data, size_t size)
{
	// transfer(data, nullptr, size);
	int ret = transfer(data, (unsigned)size, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}
}
