/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file sensor_range_finder.hpp
 * Range finder class containing the validity checks performed on the data
 * before it is fused by the EKF.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */
#ifndef EKF_SENSOR_RANGE_FINDER_HPP
#define EKF_SENSOR_RANGE_FINDER_HPP

#include <cstdint>
#include <matrix/math.hpp>

namespace estimator
{
namespace sensor
{

struct rangeSample {
	uint64_t    time_us{};  ///< timestamp of the measurement (uSec)
	float       rng{};      ///< range (distance to ground) measurement (m)
	int8_t      quality{};  ///< Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.
};

static constexpr uint64_t RNG_MAX_INTERVAL =
	200e3;  ///< Maximum allowable time interval between range finder measurements (uSec)

class SensorRangeFinder
{
public:
	SensorRangeFinder() = default;
	~SensorRangeFinder() = default;

	void runChecks(uint64_t current_time_us, const matrix::Dcmf &R_to_earth);
	bool isHealthy() const { return _is_sample_valid; }
	bool isDataHealthy() const { return _is_sample_ready && _is_sample_valid; }
	bool isDataReady() const { return _is_sample_ready; }
	bool isRegularlySendingData() const { return _is_regularly_sending_data; }

	void setSample(const rangeSample &sample)
	{
		_sample = sample;
		_is_sample_ready = true;
	}

	// This is required because of the ring buffer
	// TODO: move the ring buffer here
	rangeSample *getSampleAddress() { return &_sample; }

	void setPitchOffset(float new_pitch_offset)
	{
		if (fabsf(_pitch_offset_rad - new_pitch_offset) > FLT_EPSILON) {
			_sin_pitch_offset = sinf(new_pitch_offset);
			_cos_pitch_offset = cosf(new_pitch_offset);
			_pitch_offset_rad = new_pitch_offset;
		}
	}

	void setCosMaxTilt(float cos_max_tilt) { _range_cos_max_tilt = cos_max_tilt; }

	// Sensor-reported nominal min/max range. Stored as informational metadata only —
	// not used for validity gating. Consumers (e.g. control-limit calculations) read
	// these directly via getValidMinVal()/getValidMaxVal().
	void setLimits(float min_distance, float max_distance)
	{
		_rng_valid_min_val = min_distance;
		_rng_valid_max_val = max_distance;
	}

	float getValidMinVal() const { return _rng_valid_min_val; }
	float getValidMaxVal() const { return _rng_valid_max_val; }

	float getCosTilt() const { return _cos_tilt_rng_to_earth; }

	void setRange(float rng) { _sample.rng = rng; }
	float getRange() const { return _sample.rng; }

	float getDistBottom() const { return _sample.rng * _cos_tilt_rng_to_earth; }

	void setDataReadiness(bool is_ready) { _is_sample_ready = is_ready; }
	void setValidity(bool is_valid) { _is_sample_valid = is_valid; }

private:
	void updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth);

	void updateValidity(uint64_t current_time_us);
	void updateDtDataLpf(uint64_t current_time_us);
	bool isSampleOutOfDate(uint64_t current_time_us) const;
	bool isDataContinuous() const { return _dt_data_lpf < 2e6f; }
	bool isTiltOk() const { return _cos_tilt_rng_to_earth > _range_cos_max_tilt; }
	bool isQualityOk() const { return _sample.quality != 0; }

	rangeSample _sample{};

	bool _is_sample_ready{};	///< true when new range finder data has fallen behind the fusion time horizon and is available to be fused
	bool _is_sample_valid{};	///< true if the most recent sample passed the validity checks
	bool _is_regularly_sending_data{false}; ///< true if the interval between two samples is less than the maximum expected interval

	/*
	 * Data regularity check
	 */
	static constexpr float _dt_update{0.01f}; 	///< delta time since last ekf update TODO: this should be a parameter
	float _dt_data_lpf{};	///< filtered value of the delta time elapsed since the last range measurement came into the filter (uSec)

	/*
	 * Tilt check
	 */
	float _cos_tilt_rng_to_earth{1.f};		///< 2,2 element of the rotation matrix from sensor frame to earth frame
	float _range_cos_max_tilt{0.7071f};	///< cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float _pitch_offset_rad{3.14f}; 		///< range finder tilt rotation about the Y body axis
	float _sin_pitch_offset{0.0f}; 		///< sine of the range finder tilt rotation about the Y body axis
	float _cos_pitch_offset{-1.0f}; 		///< cosine of the range finder tilt rotation about the Y body axis

	/*
	 * Sensor-reported nominal range limits (informational; consumed by control-limit code).
	 */
	float _rng_valid_min_val{};	///< minimum distance the rangefinder can measure (m)
	float _rng_valid_max_val{};	///< maximum distance the rangefinder can measure (m)
};

} // namespace sensor
} // namespace estimator
#endif // !EKF_SENSOR_RANGE_FINDER_HPP
