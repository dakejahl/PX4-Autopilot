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

#include "Display.hpp"
#include "images.h"
#include <version/version.h>

static constexpr uint16_t FONT_10_HEIGHT = 13; // ArialMT_Plain_10
static constexpr uint16_t FONT_16_HEIGHT = 18; // ArialMT_Plain_16

app_state_s _state = {};
watts_battery_status_s _battery_status;

void booting_loading_page(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Splash screen
	display->drawXbm(x, y, Watts_Logo_Width, Watts_Logo_Height, Watts_Logo_Bits);

	// Loading progress bar
	uint16_t progress_bar_width = 80;
	uint16_t progress_bar_height = 5;
	uint16_t x_loading = (display->getWidth() / 2) - (progress_bar_width / 2);
	uint16_t y_loading = display->getHeight() - 7;
	display->drawProgressBar(x_loading, y_loading, progress_bar_width, progress_bar_height, _state.progress);
}

// Main page -- % remaining, voltage, current
void running_page_1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Bump the battery image up slightly so that it doesn't cover the indicators
	y = y - 3;

	// Battery Icon
	display->drawXbm(x, y, Watts_BatteryBorder_Width, Watts_BatteryBorder_Height, Watts_BatteryBorder_bits);

	uint32_t remaining = _battery_status.state_of_charge;
	if (remaining > 95) {
		display->drawXbm(x, y, Watts_BatteryBars5_Width, Watts_BatteryBars5_Height, Watts_BatteryBars5_bits);

	} else if (remaining > 75) {
		display->drawXbm(x, y, Watts_BatteryBars4_Width, Watts_BatteryBars4_Height, Watts_BatteryBars4_bits);

	} else if (remaining > 55) {
		display->drawXbm(x, y, Watts_BatteryBars3_Width, Watts_BatteryBars3_Height, Watts_BatteryBars3_bits);

	} else if (remaining > 25) {
		display->drawXbm(x, y, Watts_BatteryBars2_Width, Watts_BatteryBars2_Height, Watts_BatteryBars2_bits);

	} else {
		display->drawXbm(x, y, Watts_BatteryBars1_Width, Watts_BatteryBars1_Height, Watts_BatteryBars1_bits);
	}

	// Text
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->setFont(ArialMT_Plain_16);

	uint16_t y_off = FONT_16_HEIGHT;

	char buffer[64] = {};

	// Current
	float current = (int(_battery_status.current_filtered * 10))/10; // Fixes negative 0
	display->drawStringf(x, y + y_off, buffer, "%.1fA", double(current));

	// Next line
	y_off += FONT_16_HEIGHT;

	// Voltage
	display->drawStringf(x, y + y_off, buffer, "%.1fV", double(_battery_status.voltage));

	// Percent remaining
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128 - 5, y + 25, buffer, "%u%%", int(_battery_status.state_of_charge));
}

// Battery health page
void running_page_2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	display->setFont(ArialMT_Plain_16);

	uint16_t y_off = 0;
	char buffer[64] = {};

	// State of health
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y + y_off, "Health:");
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128, y + y_off, buffer, "%u%%", int(_battery_status.state_of_health));

	y_off += FONT_16_HEIGHT;

	// Temperature
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y + y_off, "Temp:");
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128, y + y_off, buffer, "%uF", int(_battery_status.temperature_pcb * 1.8f + 32));

	y_off += FONT_16_HEIGHT;

	// Cyle Count
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y + y_off, "Cycles:");
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128, y + y_off, buffer, "%u", int(_battery_status.cycle_count));
}

// Cell voltages page
void running_page_3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->setFont(ArialMT_Plain_10);

	float cells[12] = {};
	memcpy(cells, _battery_status.cell_voltages, sizeof(_battery_status.cell_voltages));

	uint16_t y_off = 2;
	char buffer[64] = {};

	display->drawStringf(x, y + y_off, buffer, "%8.2f  %8.2f  %8.2f", (double)cells[0], (double)cells[1], (double)cells[2]);

	y_off += FONT_10_HEIGHT;

	display->drawStringf(x, y + y_off, buffer, "%8.2f  %8.2f  %8.2f", (double)cells[3], (double)cells[4], (double)cells[5]);

	y_off += FONT_10_HEIGHT;

	display->drawStringf(x, y + y_off, buffer, "%8.2f  %8.2f  %8.2f", (double)cells[6], (double)cells[7], (double)cells[8]);

	y_off += FONT_10_HEIGHT;

	display->drawStringf(x, y + y_off, buffer, "%8.2f  %8.2f  %8.2f", (double)cells[9], (double)cells[10], (double)cells[11]);
}

// Version information
void running_page_4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	char buffer[64] = {};

	display->setFont(ArialMT_Plain_10);

	uint16_t y_off = 2;

	// nsh> ver all
	// HW arch: PX4_FMU_V5X
	// HW type: V5X02
	// HW version: 0x00000000
	// HW revision: 0x00000002
	// FW git-hash: 1c8ab2a0d7db2d14a6f320ebd8766b5ffaea28fa
	// FW version: Release 1.13.3 (17630207)
	// OS: NuttX
	// OS version: Release 11.0.0 (184549631)
	// OS git-hash: 4a1dd8680cd29f51fb0fe66dcfbf6f69bec747cf
	// Build datetime: Mar 14 2023 01:45:46
	// Build uri: localhost
	// Build variant: default
	// Toolchain: GNU GCC, 9.3.1 20200408 (release)
	// PX4GUID: 000200000000203335375942500f001f0035
	// MCU: STM32F76xxx, rev. Z

	// Page title
	display->setTextAlignment(TEXT_ALIGN_CENTER);
	display->drawString(x + 64, y + y_off, "Version Information");

	y_off += FONT_10_HEIGHT;

	// Underline
	// display->drawLine(x + 18, y + y_off, x + 128 - 18, y + y_off);
	// y_off += FONT_10_HEIGHT / 4;

	// Software version
	unsigned fwver = px4_firmware_version();
	unsigned major = (fwver >> (8 * 3)) & 0xFF;
	unsigned minor = (fwver >> (8 * 2)) & 0xFF;
	unsigned patch = (fwver >> (8 * 1)) & 0xFF;

	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y + y_off, "SW:");
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128, y + y_off, buffer, "%u.%u.%u", major, minor, patch);

	y_off += FONT_10_HEIGHT;

	// Hardware version
	char uuid[25]; // 2 characters per hex byte + null terminator
	board_get_mfguid_formated(uuid, sizeof(uuid)); //STM32 UUID is 12 bytes -- 203337314d435004001d003a

	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y + y_off, "SN:");
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->drawStringf(x + 128, y + y_off, buffer, "%.*s", 12, uuid);
	y_off += FONT_10_HEIGHT;
	display->drawStringf(x + 128, y + y_off, buffer, "%.*s", 12, &uuid[12]);


	// We don't need to use both BQ34 SN and STM32 UUID
	// Serial Number
	// display->setTextAlignment(TEXT_ALIGN_LEFT);
	// display->drawString(x, y + y_off, "SN:");
	// display->setTextAlignment(TEXT_ALIGN_RIGHT);
	// display->drawStringf(x + 128, y + y_off, buffer, "123456"); // TODO: serial from bq34
}

// Page frames
FrameCallback booting_pages[] = { booting_loading_page, running_page_1 };
int booting_page_count = sizeof(booting_pages)/sizeof(booting_pages[0]);

FrameCallback running_pages[] = { running_page_1, running_page_2, running_page_3, running_page_4 };
int running_page_count = sizeof(running_pages)/sizeof(running_pages[0]);

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

Display::Display() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::I2C2),
	_cycle_perf(perf_alloc(PC_ELAPSED, "Display: single-sample"))
{}

Display::~Display()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int Display::init()
{
	PX4_INFO("Initializing Display");

	static constexpr uint8_t I2C_ADDR = 0x3D;
	_display_interface = new SSD1306_I2C(I2C_ADDR);
	_display_ui = new OLEDDisplayUi(_display_interface);

	_display_ui->setTargetFPS(60);
	_display_ui->setActiveSymbol(activeSymbol);
	_display_ui->setInactiveSymbol(inactiveSymbol);
	_display_ui->setIndicatorPosition(BOTTOM);
	_display_ui->setIndicatorDirection(LEFT_RIGHT); // Defines where the first frame is located in the bar.
	_display_ui->setFrameAnimation(SLIDE_LEFT);
	_display_ui->setFrames(booting_pages, booting_page_count);

	_display_ui->init(); // Initialising the UI will init the display too.

	// Set it up for initial bootup
	_display_ui->disableAllIndicators();
	_display_ui->disableAutoTransition();
	_display_ui->setTimePerTransition(0);
	_display_interface->flipScreenVertically();

	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);
	return PX4_OK;
}

void Display::Run()
{
	if (should_exit()) {
		PX4_INFO("exiting");
		exit_and_cleanup();
		return;
	}

	_battery_sub.update(&_battery_status);

	uint8_t previous_state = _state.state;
	_app_state_sub.update(&_state);

	if (_state.state != previous_state) {
		if (_state.state == app_state_s::BOOTING) {
			_display_ui->switchToFrame(0);

		} else if (_state.state == app_state_s::SHOW_INFO) {
			_display_ui->switchToFrame(1);

		} else if (_state.state == app_state_s::RUNNING) {
			_display_interface->clear();
			_display_ui->enableAllIndicators();
			_display_ui->setFrames(running_pages, running_page_count);

		} else if (_state.state == app_state_s::SHUTDOWN) {
			// Shut down imminenet, disable OLED
			_display_interface->clear();
			_display_interface->display();
			ScheduleClear();
			PX4_INFO("exiting");
			return;
		}
	}

	perf_begin(_cycle_perf);

	button_pressed_s button = {};
	if (_button_pressed_sub.update(&button)) {
		if (_state.state == app_state_s::RUNNING) {
			_display_ui->nextFrame();
		}
	}

	_display_ui->update();

	perf_end(_cycle_perf);
}

int Display::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "command")) {
		if (is_running()) {
			// return _object.load()->diagnostics();
			return PX4_ERROR;
		}
		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int Display::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Display driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("display", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("command", "custom command goes here");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int Display::task_spawn(int argc, char *argv[])
{
	Display *instance = new Display();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int ssd1306_display_main(int argc, char *argv[])
{
	return Display::main(argc, argv);
}
