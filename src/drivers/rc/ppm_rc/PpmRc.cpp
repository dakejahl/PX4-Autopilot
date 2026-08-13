/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "PpmRc.hpp"

#include <px4_arch/io_timer.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

ModuleBase::Descriptor PpmRc::desc{task_spawn, custom_command, print_usage};

PpmRc::PpmRc() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PpmRc::~PpmRc()
{
	if (_channel >= 0) {
		up_input_capture_set(_channel, Disabled, 0, nullptr, nullptr);
		io_timer_unallocate_channel(_channel);
	}
}

bool PpmRc::init()
{
	// The pin is whichever output the user assigned PPM_Input to, so a different
	// pin is a parameter change rather than a firmware build.
	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		param_t function_handle = param_find(param_name);
		int32_t function;

		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
			if (function == OUTPUT_FUNCTION_PPM_INPUT) {
				_channel = i;
			}
		}
	}

	if (_channel == -1) {
		PX4_WARN("no channel assigned the PPM Input function");
		return false;
	}

	// PPM carries its timing in both edges, and the capture hardware timestamps
	// them far more tightly than an EXTI interrupt would.
	int ret = up_input_capture_set(_channel, Both, 0, &PpmRc::capture_trampoline, this);

	if (ret != PX4_OK) {
		PX4_ERR("capture channel %d unavailable (%i)", _channel, ret);
		_channel = -1;
		return false;
	}

	if (!_input_rc_pub.advertise()) {
		return false;
	}

	ScheduleOnInterval(SCHEDULE_INTERVAL);

	return true;
}

void PpmRc::capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
			       uint32_t overflow)
{
	// Runs in the timer ISR. edge_time is already on the HRT time base, so the
	// decoder never has to reason about the capture timer's own counter.
	static_cast<PpmRc *>(context)->_decoder.edge(edge_time, overflow != 0);
}

void PpmRc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// Snapshot the decoder, which the capture ISR writes underneath us
	uint16_t values[PPMDecoder::MAX_CHANNELS];
	irqstate_t flags = px4_enter_critical_section();
	const unsigned channel_count = _decoder.channel_count();
	const uint64_t last_decode = _decoder.last_decode_time();
	const uint16_t frame_length = _decoder.frame_length();
	memcpy(values, _decoder.channels(), sizeof(values));
	px4_leave_critical_section(flags);

	if (last_decode == _last_published_decode || channel_count < 4) {
		return;
	}

	_last_published_decode = last_decode;

	input_rc_s input_rc{};
	input_rc.timestamp_last_signal = last_decode;
	input_rc.channel_count = math::min(channel_count, (unsigned)input_rc_s::RC_INPUT_MAX_CHANNELS);
	input_rc.rc_ppm_frame_length = frame_length;
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
	input_rc.rssi = -1;

	for (unsigned i = 0; i < input_rc.channel_count; i++) {
		input_rc.values[i] = values[i];
	}

	input_rc.timestamp = hrt_absolute_time();
	_input_rc_pub.publish(input_rc);
	_frames_published++;
}

int PpmRc::task_spawn(int argc, char *argv[])
{
	PpmRc *instance = new PpmRc();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PpmRc::print_status()
{
	PX4_INFO("capture channel: %d", _channel);
	PX4_INFO("channels decoded: %u", _decoder.channel_count());
	PX4_INFO("frames published: %" PRIu32, _frames_published);
	PX4_INFO("last frame: %.3f s ago",
		 (double)(hrt_absolute_time() - _decoder.last_decode_time()) / 1e6);
	return 0;
}

int PpmRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PpmRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Decodes PPM (CPPM / PPM-sum) RC input from a timer capture channel.

Assign the `PPM Input` output function to the pin the receiver is wired to
(e.g. `PWM_MAIN_FUNC5`), then enable the driver with `RC_PPM_ENABLE`.

Unlike the PPM decoder built into the HRT, the pin is not fixed at build time.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ppm_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ppm_rc_main(int argc, char *argv[])
{
	return ModuleBase::main(PpmRc::desc, argc, argv);
}
