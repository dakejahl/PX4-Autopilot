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

void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state)
{
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	display->setFont(ArialMT_Plain_10);
	// display->drawString(128, 0, String(millis()));
	display->drawString(128, 0, "420");
}

void drawBooting(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	int16_t x_offset = (display->getWidth()  / 2)  - (Watts_Logo_Width / 2);
	int16_t y_offset = (display->getHeight()  / 2)  - (Watts_Logo_Height / 2);

	display->drawXbm(x + x_offset, y + y_offset, Watts_Logo_Width, Watts_Logo_Height, Watts_Logo_Bits);
}

void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	int16_t x_offset = (display->getWidth()  / 2)  - (Watts_Logo_Width / 2);
	int16_t y_offset = (display->getHeight()  / 2)  - (Watts_Logo_Height / 2);

	display->drawXbm(x + x_offset, y + y_offset, Watts_Logo_Width, Watts_Logo_Height, Watts_Logo_Bits);
}

void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Demonstrates the 3 included default sizes. The fonts come from SSD1306Fonts.h file
	// Besides the default fonts there will be a program to convert TrueType fonts into this format
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->setFont(ArialMT_Plain_10);
	display->drawString(0 + x, 10 + y, "Arial 10");

	display->setFont(ArialMT_Plain_16);
	display->drawString(0 + x, 20 + y, "Arial 16");

	display->setFont(ArialMT_Plain_24);
	display->drawString(0 + x, 34 + y, "Arial 24");
}

void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Center point of OLED
	int16_t x_center = (display->getWidth()  / 2);
	int16_t y_center = (display->getHeight()  / 2);

	// Text alignment demo
	display->setFont(ArialMT_Plain_10);

	// The coordinates define the left starting point of the text
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->drawString(x, y, "Left aligned (0,10)");

	// The coordinates define the center of the text
	display->setTextAlignment(TEXT_ALIGN_CENTER);
	display->drawString(x + x_center, y + y_center, "Center aligned (64,22)");

	// The coordinates define the right end of the text
	display->setTextAlignment(TEXT_ALIGN_RIGHT);
	// display->drawString(x + display->getWidth(), y + display->getHeight() - 1, "Right aligned (128,33)");
	display->drawString(x + display->getWidth(), y + 45, "Right aligned (128,33)");

}

void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	// Demo for drawStringMaxWidth:
	// with the third parameter you can define the width after which words will be wrapped.
	// Currently only spaces and "-" are allowed for wrapping
	display->setTextAlignment(TEXT_ALIGN_LEFT);
	display->setFont(ArialMT_Plain_10);
	display->drawStringMaxWidth(0 + x, 10 + y, 128, "Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore what the fuck is this text even saying.");
	// display->drawStringMaxWidth(x, y, 128, "Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore what the fuck is this text even saying.");

}

void drawFrame5(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	int16_t x_offset = (display->getWidth()  / 2)  - (Terran_Logo_Width / 2);
	int16_t y_offset = (display->getHeight()  / 2)  - (Terran_Logo_Height / 2);

	display->drawXbm(x + x_offset, y_offset, Terran_Logo_Width, Terran_Logo_Height, Terran_Logo_Bits);
}

int booting_page_count = 1;
FrameCallback booting_pages[] = { drawBooting };


int running_page_count = 5;
FrameCallback running_pages[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4, drawFrame5 };


// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { msOverlay };
int overlaysCount = 1;

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

	uint8_t previous_state = _state.state;
	_app_state_sub.update(&_state);

	if (_state.state != previous_state) {
		if (_state.state == app_state_s::RUNNING) {
			_display_ui->enableAllIndicators();
			_display_ui->setTimePerTransition(0);
			_display_interface->clear();
			_display_ui->setFrames(running_pages, running_page_count);
			_display_ui->setOverlays(overlays, overlaysCount);

		} else if (_state.state == app_state_s::SHUTDOWN) {
			// Shut down imminenet, disable OLED
			_display_interface->clear();
			_display_interface->display();
			ScheduleClear();
			PX4_INFO("Exiting");
			return;
		}
	}

	// Handle button presses during running
	if (_state.state == app_state_s::RUNNING) {
		button_pressed_s button = {};
		if (_button_pressed_sub.update(&button)) {
			_display_ui->nextFrame();
		}
	} else if (_state.state == app_state_s::BOOTING) {
		// Update progress
	}

	perf_begin(_cycle_perf);

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
