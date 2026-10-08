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
 * @file AccelerationLimits.hpp
 *
 * Limits on the horizontal acceleration that keep the vehicle a distance from the obstacles
 * around it. Each obstacle caps the speed towards its nearest point at what the vehicle can still
 * stop from before the distance kept, so a command into an obstacle stops short of it and a
 * command past it slides along it. Nothing turns the vehicle towards a direction of its own choosing.
 *
 * All directions are in one horizontal frame, sector k centred on k * kSectorWidth radians.
 */

#pragma once

#include <matrix/math.hpp>

namespace collision_prevention
{

static constexpr int kSectors = 72;
static constexpr float kSectorWidth = 2.f * M_PI_F / kSectors;

struct PolarObstacles {
	float distance[kSectors]; ///< [m] nearest obstacle in the sector, INFINITY if none in range, NAN if no data
	float range[kSectors];    ///< [m] how far the sector was observed, used where distance is INFINITY
};

/** A cap on the speed along a horizontal direction, for something in the vehicle's path */
struct PathLimit {
	matrix::Vector2f direction; ///< unit
	float max_speed;            ///< [m/s]
};

/** Scratch for limitAcceleration(), too large for a work queue's stack */
struct LimitWorkspace {
	static constexpr int kMaxPathLimits = 2;
	// a limit per sector, three along the motion, the path limits, backing out and a slide
	static constexpr int kMaxConstraints = kSectors + 3 + kMaxPathLimits + 2;
	matrix::Vector2f directions[kMaxConstraints];
	float bounds[kMaxConstraints];
	float estimate_bounds[kMaxConstraints]; ///< the same from the estimated velocity alone
	matrix::Vector2f corrections[kMaxConstraints];
};

struct LimitConfig {
	float distance{1.f};         ///< [m] kept from every obstacle
	float max_acceleration{3.f}; ///< [m/s^2] braking
	float max_jerk{10.f};        ///< [m/s^3] braking
	float gain{1.f};             ///< [1/s] acceleration per m/s the speed is above what is allowed
	float delay{0.f};            ///< [s] sensor and tracking delay, flown before braking starts
	float push_time{1.f};        ///< [s] an obstacle inside the distance is backed away from at its depth per this
};

/** Sector whose centre is closest to the direction angle [rad] */
int sectorOf(float angle);

/** Unit vector along the centre of a sector */
matrix::Vector2f sectorDirection(int sector);

/**
 * Acceleration close to the requested one such that the speed towards the nearest point of each
 * obstacle stays within what the vehicle can stop from before config.distance, and backs out of
 * anything already inside it. Along the velocity and the acceleration, the speed also stays
 * within what it can stop from at the end of what the sensors have seen clear, or, where they
 * have not looked, as if something stood just past config.distance: the pilot may fly there, slowly.
 * Collision Prevention itself never slides the vehicle there: a slide towards unseen space stops.
 *
 * @param velocity setpoint the acceleration is integrated into
 * @param velocity_estimate of the vehicle, which can lag the setpoint; the faster of the two counts
 * @param path_limits further caps on the speed along a direction
 */
matrix::Vector2f limitAcceleration(const matrix::Vector2f &acceleration, const matrix::Vector2f &velocity,
				   const matrix::Vector2f &velocity_estimate, const PolarObstacles &obstacles, const LimitConfig &config,
				   LimitWorkspace &workspace, const PathLimit *path_limits = nullptr, int path_limit_count = 0);

} // namespace collision_prevention
