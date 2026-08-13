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

/**
 * @file PpmRc.hpp
 *
 * PPM (CPPM / PPM-sum) RC input on a generic timer capture channel.
 *
 * The legacy PPM path lives inside the HRT, which pins it to one timer, one
 * compare channel and one pin chosen at build time. This driver instead takes
 * whichever FMU PWM pin the user assigns the PPM_Input output function to, the
 * same mechanism rpm_capture and camera_capture use, so the pin is a parameter
 * rather than a firmware build.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <lib/rc/ppm_decoder.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>

using namespace time_literals;

class PpmRc : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	PpmRc();
	~PpmRc() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	/** Output function value for PPM_Input, see mixer_module/output_functions.yaml */
	static constexpr int32_t OUTPUT_FUNCTION_PPM_INPUT{2076};

	/** PPM frames arrive at roughly 45 Hz, poll comfortably faster than that */
	static constexpr uint32_t SCHEDULE_INTERVAL{10_ms};

	void Run() override;

	static void capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
				       uint32_t overflow);

	PPMDecoder _decoder{};

	int _channel{-1};
	uint64_t _last_published_decode{0};
	uint32_t _frames_published{0};

	uORB::PublicationMulti<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};
};
