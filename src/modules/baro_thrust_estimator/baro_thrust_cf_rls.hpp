/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file baro_thrust_cf_rls.hpp
 *
 * CF + parallel-RLS estimator for barometer thrust compensation.
 * Pure math — no uORB, no PX4 module infrastructure.
 */

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/math.hpp>
#include <geo/geo.h>

#include <cmath>
#include <float.h>

class BaroThrustCfRls {
public:
	static constexpr int NUM_TAU_CANDIDATES = 5;
	static constexpr float TAU_CANDIDATES[NUM_TAU_CANDIDATES] = {0.0f, 0.02f, 0.05f, 0.1f, 0.2f};

	static constexpr float RLS_LAMBDA = 0.998f;
	static constexpr float RLS_P_INIT = 100.f;
	static constexpr float CONVERGENCE_VAR_THR = 3.f;
	static constexpr float CONVERGENCE_ERR_THR = 0.5f;
	static constexpr float MIN_THRUST_EXCITATION = 0.05f;
	static constexpr float MIN_ESTIMATION_TIME_S = 30.f;
	static constexpr float K_STABILITY_TIME_S = 10.f;
	static constexpr float CONVERGENCE_HOLD_TIME_S = 10.f;
	static constexpr float K_STABILITY_DIFF_THR = 0.5f;
	static constexpr float BANK_SWITCH_HYSTERESIS = 0.9f;
	static constexpr float CF_BANDWIDTH_HZ = 0.05f;
	static constexpr float ERROR_VAR_INIT = 10.f;

	struct RlsEstimator {
		float theta[2]{};       // [K, bias]
		float P[2][2]{};        // 2x2 covariance
		float error_var{ERROR_VAR_INIT};
		AlphaFilter<float> thrust_lpf{};
		float tau{0.f};

		void reset(float tau_val, float p_init);
		void update(float residual, float thrust_raw, float dt, float lambda);
	};

	void reset();

	/**
	 * Convert body-frame specific force (including gravity, FRD) to
	 * altitude-up linear acceleration.
	 *
	 * PX4 convention: vehicle_acceleration.xyz is specific force in FRD body
	 * frame. At rest level this is approximately {0, 0, -g}.
	 */
	static float computeAccelUp(const matrix::Vector3f &accel_body, const matrix::Quatf &attitude);

	/**
	 * Update complementary filter and return residual (baro - accel prediction).
	 * @param accel_up  Upward linear acceleration from computeAccelUp()
	 * @return CF residual [m], or 0 on first call (initialization)
	 */
	float updateCf(float baro_alt, float accel_up, float dt);

	void updateEstimators(float residual, float thrust, float dt);
	void checkConvergence(float elapsed_since_start_s, float dt);
	int selectBestBank() const;

	bool converged() const { return _converged; }
	bool convergedLocked() const { return _converged_locked; }
	int bestBankIdx() const { return _best_bank_idx; }
	const RlsEstimator &bestBank() const { return _bank[_best_bank_idx]; }
	const RlsEstimator &bank(int i) const { return _bank[i]; }
	float thrustStd() const { return sqrtf(fmaxf(_thrust_var.getState(), 0.f)); }

private:
	RlsEstimator _bank[NUM_TAU_CANDIDATES]{};
	int _best_bank_idx{0};

	float _cf_alt{0.f};
	float _cf_vel{0.f};
	bool _cf_initialized{false};

	AlphaFilter<float> _k_est_smoothed{};
	float _k_stable_elapsed_s{0.f};
	bool _converged{false};
	bool _converged_locked{false};
	float _converged_elapsed_s{0.f};

	AlphaFilter<float> _thrust_mean{};
	AlphaFilter<float> _thrust_var{};
};
