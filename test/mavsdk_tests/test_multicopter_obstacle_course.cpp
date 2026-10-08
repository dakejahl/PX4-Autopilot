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


// SIH flies the obstacle course (SIH_WLD_TYPE 2) with the simulated range image sensor and the
// obstacle map on, set through the environment in the config: a fence, then a roof. The vehicle
// is flown north in Position mode with stick input only. It pushes forward into the fence and
// climbs until Collision Prevention lets it over, flies on under the roof, and climbs into it,
// straight up and then forward. Every ground truth sample is checked against the world library's
// boxes.

#include "autopilot_tester.h"

#include <lib/obstacle_sim/obstacle_sim.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>

using namespace std::chrono_literals;

namespace
{

constexpr unsigned kStickRateHz = 50;
constexpr std::chrono::milliseconds kStickPeriod{1000 / kStickRateHz};
constexpr float kTakeoffAltitudeM = 1.f;
// the map's 0.15 m voxels and the estimate's offset from the truth
constexpr float kToleranceM = 0.25f;
// a limit that is what stopped the vehicle leaves it within this of where it holds
constexpr float kEngagedMarginM = 0.5f;
constexpr float kHeldSpeedMS = 0.5f;
constexpr int kFence = 0;
constexpr int kRoof = 1;

class AutopilotTesterObstacleCourse : public AutopilotTester
{
public:
	void load_world()
	{
		auto params = getParams();
		obstacle_sim::WorldConfig config{};
		config.type = static_cast<obstacle_sim::WorldType>(params->get_param_int("SIH_WLD_TYPE").second);
		REQUIRE(config.type == obstacle_sim::WorldType::Course);
		_world.configure(config);
		REQUIRE(_world.boxCount() == 2);

		const double lat0 = params->get_param_float("SIH_LOC_LAT0").second;
		const double lon0 = params->get_param_float("SIH_LOC_LON0").second;
		_ground_altitude_m = params->get_param_float("SIH_LOC_H0").second;
		_ct = std::make_unique<CoordinateTransformation>(CoordinateTransformation::GlobalCoordinate{lat0, lon0});

		_cp_dist_m = params->get_param_float("CP_DIST").second;
		_cp_dist_v_m = params->get_param_float("CP_DIST_V").second;
		_half_height_m = 0.5f * params->get_param_float("OMAP_VEH_HGT").second;
		REQUIRE(_cp_dist_m > 0.f);
	}

	const obstacle_sim::Box &box(int index) const { return _world.box(index); }

	void start_sampling()
	{
		CHECK(getTelemetry()->set_rate_ground_truth(50) == Telemetry::Result::Success);
		CHECK(getTelemetry()->set_rate_position_velocity_ned(50) == Telemetry::Result::Success);
		_ground_truth_handle = getTelemetry()->subscribe_ground_truth([this](Telemetry::GroundTruth g) {
			const auto local = _ct->local_from_global({g.latitude_deg, g.longitude_deg});
			const float point[3] {(float)local.north_m, (float)local.east_m, (float)(_ground_altitude_m - g.absolute_altitude_m)};

			std::lock_guard<std::mutex> lock(_mutex);
			_north = point[0];

			if (!_sampling) {
				return;
			}

			_samples++;
			int index = -1;
			const float distance = _world.nearestBox(point, &index);

			if (index >= 0 && distance < _min_distance_m[index]) {
				_min_distance_m[index] = distance;
				_closest[index][0] = point[0];
				_closest[index][1] = point[1];
				_closest[index][2] = point[2];
			}

			_max_north_m = std::max(_max_north_m, point[0]);
			const obstacle_sim::Box &roof = _world.box(kRoof);

			if (point[0] > roof.min[0] && point[0] < roof.max[0]) {
				_max_altitude_under_roof_m = std::max(_max_altitude_under_roof_m, -point[2]);
			}
		});
	}

	// the callback holds this, so it goes before the tester does
	void stop_sampling()
	{
		getTelemetry()->unsubscribe_ground_truth(_ground_truth_handle);
	}

	void set_sampling(bool on)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_sampling = on;
	}

	float north()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _north;
	}

	void sticks(float pitch, float roll, float throttle, float yaw, std::chrono::milliseconds duration)
	{
		for (auto t = 0ms; t < duration; t += kStickPeriod) {
			CHECK(getManualControl()->set_manual_control_input(pitch, roll, throttle, yaw) == ManualControl::Result::Success);
			sleep_for(kStickPeriod);
		}
	}

	// sticks until the vehicle is further north than north_m
	bool sticks_until_north(float pitch, float throttle, float north_m, std::chrono::milliseconds timeout)
	{
		for (auto t = 0ms; t < timeout; t += kStickPeriod) {
			if (north() > north_m) {
				return true;
			}

			sticks(pitch, 0.f, throttle, 0.f, kStickPeriod);
		}

		return false;
	}

	// forward stick, climbing only while held in front of the fence, as a pilot lets go of the
	// climb once the vehicle moves over
	bool climb_over(const obstacle_sim::Box &fence, std::chrono::milliseconds timeout)
	{
		for (auto t = 0ms; t < timeout; t += kStickPeriod) {
			const auto velocity = getTelemetry()->position_velocity_ned().velocity;
			const bool held = std::hypot(velocity.north_m_s, velocity.east_m_s) < kHeldSpeedMS;
			const float here = north();

			if (here > fence.max[0] + 1.5f) {
				return true;
			}

			sticks(1.f, 0.f, (held && here < fence.min[0]) ? 1.f : 0.5f, 0.f, kStickPeriod);
		}

		return false;
	}

	void take_off_in_position_mode()
	{
		// the stick stream has to be up before Position mode and arming are accepted
		sticks(0.f, 0.f, 0.5f, 0.f, 1s);
		REQUIRE(getManualControl()->start_position_control() == ManualControl::Result::Success);
		sticks(0.f, 0.f, 0.5f, 0.f, 1s);
		arm();

		for (auto t = 0ms; t < 20s; t += kStickPeriod) {
			if (getTelemetry()->position_velocity_ned().position.down_m < -kTakeoffAltitudeM) {
				break;
			}

			sticks(0.f, 0.f, 1.f, 0.f, kStickPeriod);
		}

		sticks(0.f, 0.f, 0.5f, 0.f, 2s);
		REQUIRE(getTelemetry()->position_velocity_ned().position.down_m < -0.8f * kTakeoffAltitudeM);
	}

	void land()
	{
		for (auto t = 0ms; t < 60s; t += kStickPeriod) {
			sticks(0.f, 0.f, 0.f, 0.f, kStickPeriod);

			if (!getTelemetry()->in_air()) {
				break;
			}
		}

		wait_until_disarmed(30s);
	}

	void check()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		const float min_distance_m = _half_height_m + _cp_dist_v_m - kToleranceM;
		const char *names[] {"fence", "roof"};

		for (int i = 0; i < 2; i++) {
			std::cout << time_str() << "Closest to the " << names[i] << " " << _min_distance_m[i] << " m, from " << _closest[i][0]
				  << ", " << _closest[i][1] << ", " << _closest[i][2] << " m" << std::endl;
			CHECK(_min_distance_m[i] > min_distance_m);
		}

		const obstacle_sim::Box &roof = _world.box(kRoof);
		const float roof_limit_m = -roof.max[2] - _half_height_m - _cp_dist_v_m;
		std::cout << time_str() << "Highest under the roof " << _max_altitude_under_roof_m << " m, the climb limit is "
			  << roof_limit_m << " m; furthest north " << _max_north_m << " m, over " << _samples << " samples" << std::endl;
		REQUIRE(_samples > 100);
		// stopped by the fence, then over it and on under the roof
		CHECK(_min_distance_m[kFence] < _cp_dist_m + kEngagedMarginM);
		CHECK(_max_north_m > roof.min[0] + 2.f);
		// the climb went until Collision Prevention held it, under the roof rather than over it
		CHECK(_max_altitude_under_roof_m > roof_limit_m - kEngagedMarginM);
		CHECK(_max_altitude_under_roof_m < -roof.max[2]);
	}

private:
	obstacle_sim::World _world{};
	std::unique_ptr<CoordinateTransformation> _ct{};
	float _ground_altitude_m{NAN};
	float _cp_dist_m{NAN};
	float _cp_dist_v_m{NAN};
	float _half_height_m{NAN};

	std::mutex _mutex;
	Telemetry::GroundTruthHandle _ground_truth_handle{};
	bool _sampling{false};
	unsigned _samples{0};
	float _north{NAN};
	float _min_distance_m[2] {INFINITY, INFINITY};
	float _closest[2][3] {};
	float _max_north_m{-INFINITY};
	float _max_altitude_under_roof_m{-INFINITY};
};

} // namespace

TEST_CASE("Collision Prevention keeps the vehicle off the fence and the roof", "[obstacle_course]")
{
	AutopilotTesterObstacleCourse tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.load_world();
	tester.store_home();
	tester.start_sampling();

	tester.take_off_in_position_mode();
	tester.set_sampling(true);

	// into the fence, which stops the vehicle short of it
	tester.sticks(1.f, 0.f, 0.5f, 0.f, 8s);
	const obstacle_sim::Box &fence = tester.box(kFence);
	CHECK(tester.north() < fence.min[0]);

	// climbing until it lets the vehicle over, then on level under the roof
	CHECK(tester.climb_over(fence, 40s));
	const obstacle_sim::Box &roof = tester.box(kRoof);
	CHECK(tester.sticks_until_north(1.f, 0.5f, roof.min[0] + 4.f, 30s));

	// Climbing into the roof, straight up and then forward. Forward only as far as the map holds
	// the roof: it reaches 4.8 m out, and pitched forward the sensor does not see a roof this close
	// overhead, so further on the space above is unknown and the climb only slowed.
	tester.sticks(0.f, 0.f, 1.f, 0.f, 8s);
	tester.sticks(1.f, 0.f, 1.f, 0.f, 2s);
	tester.set_sampling(false);

	tester.land();
	tester.stop_sampling();
	tester.check();
}
