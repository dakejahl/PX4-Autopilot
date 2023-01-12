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

#include "Display.hpp"
#include "images.h"


//////////////////////////////////////////////////////////
///////////////////// FIX THIS ///////////////////////////
//
// But how do we fix it? We need to pass function pointers
// and that is hard to do with classes... hmmm
//
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

void running_page_1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Bump the battery image up slightly so that it doesn't cover the indicators
	y = y - 3;

	// Battery
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

	static constexpr uint16_t ArialMT_Plain_16_Height = 18; // ArialMT_Plain_16

	char text_temp[64] = {};

	snprintf(text_temp, sizeof(text_temp), "%.1fAX", double(_battery_status.current));
	display->drawString(x, y + ArialMT_Plain_16_Height, text_temp);

	snprintf(text_temp, sizeof(text_temp), "%.1fV", double(_battery_status.voltage));
	display->drawString(x, y + (ArialMT_Plain_16_Height * 2), text_temp);

	snprintf(text_temp, sizeof(text_temp), "%u%%", int(_battery_status.state_of_charge));
	display->drawString(x + 86, y + 23, text_temp);
}

void running_page_2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	display->setFont(ArialMT_Plain_16);

	uint16_t vertical_offset = 0;
	char text_temp[64] = {};

	static constexpr uint16_t ArialMT_Plain_16_Height = 18; // ArialMT_Plain_16

	// State of health
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	snprintf(text_temp, sizeof(text_temp), "Health:");
	display->drawString(x, y + vertical_offset, text_temp);
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	snprintf(text_temp, sizeof(text_temp), "%u%%", int(_battery_status.state_of_health));
	display->drawString(x + 128, y + vertical_offset, text_temp);
	vertical_offset += ArialMT_Plain_16_Height;

	// Temperature
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	snprintf(text_temp, sizeof(text_temp), "Temp:");
	display->drawString(x, y + vertical_offset, text_temp);
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	snprintf(text_temp, sizeof(text_temp), "%uF", int(_battery_status.temperature_pcb * 1.8f + 32));
	display->drawString(x + 128, y + vertical_offset, text_temp);
	vertical_offset += ArialMT_Plain_16_Height;

	// Cyle Count
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	snprintf(text_temp, sizeof(text_temp), "Cycles:");
	display->drawString(x, y + vertical_offset, text_temp);
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	snprintf(text_temp, sizeof(text_temp), "%u", int(_battery_status.cycle_count));
	display->drawString(x + 128, y + vertical_offset, text_temp);
}

void running_page_3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->setFont(ArialMT_Plain_10);

	static constexpr uint16_t ArialMT_Plain_10_Height = 13; // ArialMT_Plain_10

	uint16_t vertical_offset = 2;
	char text_temp[64] = {};

	snprintf(text_temp, sizeof(text_temp), "%8.2f  %8.2f  %8.2f", (double)_battery_status.cell_voltages[0], (double)_battery_status.cell_voltages[1], (double)_battery_status.cell_voltages[2]);
	display->drawString(x, y + vertical_offset, text_temp);
	vertical_offset += ArialMT_Plain_10_Height;

	snprintf(text_temp, sizeof(text_temp), "%8.2f  %8.2f  %8.2f", (double)_battery_status.cell_voltages[3], (double)_battery_status.cell_voltages[4], (double)_battery_status.cell_voltages[5]);
	display->drawString(x, y + vertical_offset, text_temp);
	vertical_offset += ArialMT_Plain_10_Height;

	snprintf(text_temp, sizeof(text_temp), "%8.2f  %8.2f  %8.2f", (double)_battery_status.cell_voltages[6], (double)_battery_status.cell_voltages[7], (double)_battery_status.cell_voltages[8]);
	display->drawString(x, y + vertical_offset, text_temp);
	vertical_offset += ArialMT_Plain_10_Height;

	snprintf(text_temp, sizeof(text_temp), "%8.2f  %8.2f  %8.2f", (double)_battery_status.cell_voltages[9], (double)_battery_status.cell_voltages[10], (double)_battery_status.cell_voltages[11]);
	display->drawString(x, y + vertical_offset, text_temp);
}

// Page frames
int booting_page_count = 2;
FrameCallback booting_pages[] = { booting_loading_page, running_page_1 };

int running_page_count = 3;
FrameCallback running_pages[] = { running_page_1, running_page_2, running_page_3 };

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

	_display_interface = new SSD1306_I2C(); // Pass addr as arg
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
