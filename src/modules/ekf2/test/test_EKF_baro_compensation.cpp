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
 * Test thrust-based baro propwash compensation (EKF2_PCOEF_THR)
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfBaroCompensationTest : public ::testing::Test
{
public:
	EkfBaroCompensationTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
		_sensor_simulator.runSeconds(7);
	}

	void TearDown() override
	{
	}
};

TEST_F(EkfBaroCompensationTest, testThrustCompensationDefault)
{
	// With default parameter (0), thrust should have no effect on baro height
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator._baro.setThrustMagnitude(0.6f);

	const float height_before = _ekf->getPosition()(2);

	_sensor_simulator.runSeconds(10);

	const float height_after = _ekf->getPosition()(2);

	// Height should not change significantly (only normal drift)
	EXPECT_NEAR(height_after, height_before, 0.1f);
}

TEST_F(EkfBaroCompensationTest, testThrustCompensationPositive)
{
	// Set a positive correction coefficient and apply thrust
	const float ekf2_pcoef_thr = 2.0f;
	const float thrust_magnitude = 0.6f;

	parameters *params = _ekf->getParamHandle();
	params->ekf2_pcoef_thr = ekf2_pcoef_thr;

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	const float height_before = _ekf->getPosition()(2);

	_sensor_simulator._baro.setThrustMagnitude(thrust_magnitude);
	_sensor_simulator.runSeconds(20);

	const float height_after = _ekf->getPosition()(2);

	// Positive pcoef_thr increases baro_alt, which means the EKF sees a higher
	// baro measurement. The EKF state z (NED down) should decrease (altitude increases).
	// Instantaneous baro altitude offset: ekf2_pcoef_thr * thrust_magnitude = 1.2 m
	const float expected_correction = ekf2_pcoef_thr * thrust_magnitude;

	// The EKF fuses this over time; check that height moved in the expected direction
	// and the magnitude is in the right ballpark
	EXPECT_LT(height_after, height_before - 0.5f * expected_correction);
}

TEST_F(EkfBaroCompensationTest, testThrustCompensationDisabledInFixedWing)
{
	// Thrust compensation should not apply in fixed-wing mode
	const float ekf2_pcoef_thr = 2.0f;

	parameters *params = _ekf->getParamHandle();
	params->ekf2_pcoef_thr = ekf2_pcoef_thr;

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);

	const float height_before = _ekf->getPosition()(2);

	_sensor_simulator._baro.setThrustMagnitude(0.6f);
	_sensor_simulator.runSeconds(10);

	const float height_after = _ekf->getPosition()(2);

	// Height should not change significantly — fixed-wing guard disables correction
	EXPECT_NEAR(height_after, height_before, 0.1f);
}

TEST_F(EkfBaroCompensationTest, testThrustFilterSteadyState)
{
	// With a lag filter enabled, steady-state compensation should converge
	// to the same value as without filtering
	const float ekf2_pcoef_thr = 2.0f;
	const float thrust_magnitude = 0.6f;

	parameters *params = _ekf->getParamHandle();
	params->ekf2_pcoef_thr = ekf2_pcoef_thr;
	params->ekf2_pcoef_thr_tau = 0.2f;  // 200ms time constant

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	_sensor_simulator._baro.setThrustMagnitude(thrust_magnitude);

	// Run long enough for both the lag filter and EKF to converge
	_sensor_simulator.runSeconds(30);

	const float height_filtered = _ekf->getPosition()(2);

	// Compare against a fresh instance with tau=0 (no filtering)
	auto ekf_nofilter = std::make_shared<Ekf>();
	SensorSimulator sim_nofilter(ekf_nofilter);
	ekf_nofilter->init(0);
	sim_nofilter.runSeconds(0.1);
	ekf_nofilter->set_in_air_status(false);
	ekf_nofilter->set_vehicle_at_rest(true);
	sim_nofilter.runSeconds(7);

	parameters *params_nf = ekf_nofilter->getParamHandle();
	params_nf->ekf2_pcoef_thr = ekf2_pcoef_thr;
	params_nf->ekf2_pcoef_thr_tau = 0.0f;

	ekf_nofilter->set_in_air_status(true);
	ekf_nofilter->set_vehicle_at_rest(false);
	sim_nofilter._baro.setThrustMagnitude(thrust_magnitude);
	sim_nofilter.runSeconds(30);

	const float height_nofilter = ekf_nofilter->getPosition()(2);

	// Steady-state should match within tolerance (EKF bias estimation noise)
	EXPECT_NEAR(height_filtered, height_nofilter, 0.3f);
}

TEST_F(EkfBaroCompensationTest, testThrustFilterTransientDamping)
{
	// Verify that the lag filter damps the immediate response to a thrust step.
	// After a thrust step, the filtered compensation at 1 second should be less
	// than the steady-state value, demonstrating that the filter is smoothing.
	const float ekf2_pcoef_thr = 2.0f;
	const float tau = 0.3f;  // 300ms time constant

	parameters *params = _ekf->getParamHandle();
	params->ekf2_pcoef_thr = ekf2_pcoef_thr;
	params->ekf2_pcoef_thr_tau = tau;

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);

	// Start with zero thrust and let the filter settle
	_sensor_simulator._baro.setThrustMagnitude(0.0f);
	_sensor_simulator.runSeconds(5);

	const float height_before_step = _ekf->getPosition()(2);

	// Apply thrust step
	_sensor_simulator._baro.setThrustMagnitude(0.8f);

	// After a short time (~2*tau), the filtered thrust should still be ramping up,
	// so the height change should be less than steady-state
	_sensor_simulator.runSeconds(0.6);
	const float height_short = _ekf->getPosition()(2);

	// After a long time (>>tau), the filter converges
	_sensor_simulator.runSeconds(29.4);
	const float height_converged = _ekf->getPosition()(2);

	// The short-term height change should be smaller than the converged change
	// (filter is damping the transient)
	const float delta_short = fabsf(height_short - height_before_step);
	const float delta_converged = fabsf(height_converged - height_before_step);

	EXPECT_LT(delta_short, delta_converged * 0.9f);
}
