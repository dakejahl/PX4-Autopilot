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


#include "AccelerationLimits.hpp"

#include <lib/mathlib/math/TrajMath.hpp>
#include <mathlib/mathlib.h>

using matrix::Vector2f;

namespace collision_prevention
{

static constexpr int kDykstraPasses = 32;
// enough for the limits of neighbouring sectors to settle
static constexpr int kProjectionPasses = 32;
static constexpr float kProjectionTolerance = 1e-4f;
// [m/s] backing away from an obstacle inside the distance kept is gentle
static constexpr float kMaxPushSpeed = 1.f;
// [m/s^2] less than this is not a slide
static constexpr float kMinSlide = 0.05f;
// turned less than about 10 degrees from the stick, the acceleration is the pilot's
static constexpr float kSlideCosine = 0.985f;

static inline int wrapSector(int sector)
{
	return ((sector % kSectors) + kSectors) % kSectors;
}

int sectorOf(float angle)
{
	return wrapSector((int)floorf(angle / kSectorWidth + 0.5f));
}

Vector2f sectorDirection(int sector)
{
	const float angle = wrapSector(sector) * kSectorWidth;
	return Vector2f(cosf(angle), sinf(angle));
}

Vector2f limitAcceleration(const Vector2f &acceleration, const Vector2f &velocity, const Vector2f &velocity_estimate,
			   const PolarObstacles &obstacles, const LimitConfig &config, LimitWorkspace &workspace,
			   const PathLimit *path_limits, int path_limit_count)
{
	// Half planes a . u <= bound: one towards the nearest point of each obstacle, one along each of
	// the velocity setpoint, the estimated velocity and the acceleration for the room the sensors
	// have seen that way, one per path limit, and one backing out.
	// bounds from the estimated velocity alone can never contradict each other
	Vector2f *directions = workspace.directions;
	float *bounds = workspace.bounds;
	float *estimate_bounds = workspace.estimate_bounds;
	int count = 0;
	int push = -1;

	// the speed along u approaches max_speed, and brakes above it; the faster of the setpoint and
	// the vehicle counts, as either can be the one closing in
	const auto add = [&](const Vector2f & u, float max_speed) {
		const float speed = math::max(velocity.dot(u), velocity_estimate.dot(u));
		directions[count] = u;
		bounds[count] = config.gain * (max_speed - speed);
		estimate_bounds[count] = config.gain * (max_speed - velocity_estimate.dot(u));
		count++;
	};

	// as fast as the vehicle can still stop within room along u, after flying on for the delay
	const auto stopping_speed = [&](const Vector2f & u, float room) {
		const float speed = math::max(math::max(velocity.dot(u), velocity_estimate.dot(u)), 0.f);
		const float available = room - speed * config.delay;
		return (available > 0.f) ? math::trajectory::computeMaxSpeedFromDistance(config.max_jerk, config.max_acceleration,
				available, 0.f) : 0.f;
	};

	const auto distance_at = [&](int k) {
		const float distance = obstacles.distance[wrapSector(k)];
		return PX4_ISFINITE(distance) ? distance : INFINITY;
	};

	// An obstacle limits only the speed towards its nearest point. Moving along a wall keeps the
	// distance to it, though it nears the points of the wall further on, so the vehicle slides.
	Vector2f away{};

	for (int k = 0; k < kSectors; k++) {
		const float distance = obstacles.distance[k];

		if (!PX4_ISFINITE(distance) || distance > distance_at(k - 1) || distance > distance_at(k + 1)) {
			continue;
		}

		const Vector2f u = sectorDirection(k);
		const float room = distance - config.distance;
		add(u, stopping_speed(u, room));
		// back away from what is already inside the distance kept, so drift cannot creep closer
		away -= u * (math::max(-room, 0.f) / config.push_time);
	}

	// Where the vehicle goes, as fast as if something stood just past the distance kept where no
	// sensor has looked, or at the end of what the sensors have seen clear. The tighter of the two
	// sectors either side of the direction counts.
	const auto room_along = [&](const Vector2f & v) {
		const float position = atan2f(v(1), v(0)) / kSectorWidth;
		const int below = (int)floorf(position);
		float room = INFINITY;

		for (int k = below; k <= below + 1; k++) {
			const int sector = wrapSector(k);
			// seen clear never allows less than not seen at all
			room = math::min(room, __builtin_isnan(obstacles.distance[sector]) ? config.distance
					 : math::max(obstacles.range[sector] - config.distance, config.distance));
		}

		return room;
	};

	const auto unseen_along = [&](const Vector2f & v) {
		const int below = (int)floorf(atan2f(v(1), v(0)) / kSectorWidth);
		return __builtin_isnan(obstacles.distance[wrapSector(below)])
		       || __builtin_isnan(obstacles.distance[wrapSector(below + 1)]);
	};

	const Vector2f motions[3] {velocity, velocity_estimate, acceleration};

	for (const Vector2f &motion : motions) {
		if (motion.longerThan(FLT_EPSILON)) {
			const Vector2f u = motion.normalized();
			add(u, stopping_speed(u, room_along(motion)));
		}
	}

	for (int i = 0; i < path_limit_count && i < LimitWorkspace::kMaxPathLimits; i++) {
		add(path_limits[i].direction, path_limits[i].max_speed);
	}

	// One velocity summed over everything inside the distance, so walls either side cancel
	// instead of contradicting.
	if (away.longerThan(kMaxPushSpeed)) {
		away = away.normalized() * kMaxPushSpeed;
	}

	if (away.longerThan(FLT_EPSILON)) {
		const Vector2f u = -away.normalized();
		push = count;
		directions[count] = u;
		bounds[count] = config.gain * (velocity_estimate.dot(u) + away.norm()) * -1.f;
		estimate_bounds[count] = bounds[count];
		count++;
	}

	// Dykstra's projection gets near the acceleration nearest the requested one that satisfies
	// every limit, where cycling through them would stop at any that does, but closes in on it
	// slowly when many limits are nearly parallel. Cycling from there then satisfies them.
	const auto project = [&](bool with_push, const float * limit) {
		const auto skipped = [&](int i) { return !with_push && i == push; };
		Vector2f limited = acceleration;
		Vector2f *corrections = workspace.corrections;

		for (int i = 0; i < count; i++) {
			corrections[i].setZero();
		}

		for (int pass = 0; pass < kDykstraPasses; pass++) {
			for (int i = 0; i < count; i++) {
				if (!skipped(i)) {
					const Vector2f shifted = limited + corrections[i];
					limited = shifted - directions[i] * math::max(shifted.dot(directions[i]) - limit[i], 0.f);
					corrections[i] = shifted - limited;
				}
			}
		}

		float largest_excess = 0.f;

		for (int pass = 0; pass < kProjectionPasses; pass++) {
			largest_excess = 0.f;

			for (int i = 0; i < count; i++) {
				const float excess = limited.dot(directions[i]) - limit[i];

				if (!skipped(i) && excess > 0.f) {
					limited -= directions[i] * excess;
					largest_excess = math::max(largest_excess, excess);
				}
			}

			if (largest_excess < kProjectionTolerance) {
				break;
			}
		}

		return (largest_excess < kProjectionTolerance) ? limited : Vector2f(NAN, NAN);
	};

	// When the limits contradict each other, backing out of the distance gives way first. Limits
	// facing each other, as the walls of a corridor, contradict when the setpoint and the vehicle
	// move differently across them; from the vehicle's velocity alone they never do, as braking
	// it satisfies them all, so braking is what is left in the end.
	const auto solve = [&]() {
		Vector2f limited = project(true, bounds);

		if (!limited.isAllFinite() && push >= 0) {
			limited = project(false, bounds);
		}

		if (!limited.isAllFinite()) {
			limited = project(false, estimate_bounds);
		}

		return limited.isAllFinite() ? limited : Vector2f(-velocity_estimate * config.gain);
	};

	Vector2f limited = solve();

	// A slide is Collision Prevention's doing, not the pilot's, so it only goes where the sensors
	// have looked: where the stick points at seen space and the limits turn the acceleration away
	// from it, onwards but towards unseen space, the vehicle stops at the obstacle instead.
	// Braking, against the stick, is no slide.
	if (limited.longerThan(kMinSlide) && acceleration.longerThan(FLT_EPSILON)
	    && limited.dot(acceleration) > 0.f
	    && limited.normalized().dot(acceleration.normalized()) < kSlideCosine
	    && unseen_along(limited) && !unseen_along(acceleration)) {
		add(limited.normalized(), 0.f);
		limited = solve();
	}

	return limited;
}

} // namespace collision_prevention
