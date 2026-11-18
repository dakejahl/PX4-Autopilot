/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FlowGyroBuffer.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/flow_raw.h>
#include <uORB/topics/paa3905_raw.h>
#include <uORB/topics/sensor_gyro.h>

namespace sensors
{

class RawFlowCapture
{
public:
	void addGyro(const sensor_gyro_s &gyro, bool continuous)
	{
		if (_gyro_device_id != gyro.device_id) {
			_buffer.reset();
			_gyro_device_id = gyro.device_id;
		}

		const float rate[3] {gyro.x, gyro.y, gyro.z};
		_buffer.push(gyro.timestamp_sample, rate, continuous);
	}

	void update()
	{
		for (unsigned i = 0; i < paa3905_raw_s::ORB_QUEUE_LENGTH; ++i) {
			if (!_pending && !(_pending = _raw_sub.update(&_raw))) {
				break;
			}

			// Wait for a gyro sample beyond the burst endpoint, but retain the raw
			// reading even if the gyro is absent or loses coverage.
			if (_buffer.newest() < _raw.timestamp_sample && hrt_elapsed_time(&_raw.timestamp_sample) < GYRO_WAIT_US) {
				break;
			}

			flow_raw_s report{};
			report.timestamp_sample = _raw.timestamp_sample;
			report.timestamp_sample_valid = true;
			report.node_timestamp_us = _raw.timestamp_sample;
			report.device_id = _raw.device_id;
			report.gyro_device_id = _gyro_device_id;
			report.frame_counter = _raw.frame_counter;
			report.interval_us = _raw.interval_us;
			report.shutter = _raw.shutter;
			report.delta_x = _raw.delta_x;
			report.delta_y = _raw.delta_y;
			report.motion = _raw.motion;
			report.observation = _raw.observation;
			report.squal_raw = _raw.squal_raw;
			report.raw_data_sum = _raw.raw_data_sum;
			const uint64_t begin = _raw.timestamp_sample >= _raw.interval_us ? _raw.timestamp_sample - _raw.interval_us : _raw.timestamp_sample;
			_buffer.integrate(begin, _raw.timestamp_sample, report.gyro_integral, report.gyro_samples);
			report.timestamp = hrt_absolute_time();
			_pub.publish(report);
			_pending = false;
		}
	}

private:
	static constexpr uint64_t GYRO_WAIT_US = 20000;
	uORB::Subscription _raw_sub{ORB_ID(paa3905_raw)};
	uORB::PublicationMulti<flow_raw_s> _pub{ORB_ID(flow_raw)};
	FlowGyroBuffer _buffer;
	paa3905_raw_s _raw{};
	uint32_t _gyro_device_id{0};
	bool _pending{false};
};

} // namespace sensors
