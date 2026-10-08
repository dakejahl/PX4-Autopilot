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

// SIH flies among procedural tree trunks (SIH_WLD_TYPE 1) with the simulated range image sensor
// and the obstacle map on, set through the environment in the config. The vehicle is flown in
// Position mode with stick input only, as an FPV pilot would: it turns to face the nearest trunk
// and pushes full forward stick for 90 s into it and on through the trees, turning the nose to
// where the vehicle goes, and turning away when stopped. Collision Prevention only limits: it
// stops the vehicle short of a trunk and slides it past one it would graze. The trunks come
// from the same world library SIH raycasts, configured from the vehicle's parameters, and every
// ground truth sample is checked against them.

#include "autopilot_tester.h"

#include <lib/obstacle_sim/obstacle_sim.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace
{

constexpr unsigned kStickRateHz = 50;
constexpr std::chrono::milliseconds kStickPeriod{1000 / kStickRateHz};
// trunks are at least 5 m tall, the ground stays below the map's height band
constexpr float kFlightAltitudeM = 2.5f;
constexpr float kTrunkSearchRadiusM = 20.f;
// the map is built in the estimated frame, which sits a few decimetres off the truth in SIH,
// and its voxels are 0.15 m
constexpr float kToleranceM = 0.5f;
// Collision Prevention keeps CP_DIST from what the map has seen before the vehicle got that close.
// A trunk counts as seen once it has been in the sensor's field of view this long, a few frames,
// further than CP_DIST but within reach of the map's sectors, where a thin trunk also spans more
// than one zone. The map forgets a trunk that leaves its window.
constexpr unsigned kSeenSamples = 15;
constexpr float kSeenRangeM = 4.f;
constexpr float kMapReachM = 4.65f;
constexpr unsigned kMaxTreesInView = 64;
// From a trunk never in view nothing keeps the vehicle but the pilot: it may come this close,
// a vehicle radius of 0.3 m and the tolerance for the estimate
constexpr float kNoContactClearanceM = 0.5f;
// pushing into the trees has to bring the vehicle this close, or Collision Prevention was never tested
constexpr float kEngagedMarginM = 1.f;
// and has to carry it at least this far from where it started, through the trees rather than stopped at the first
constexpr float kMinTravelM = 15.f;
constexpr float kYawStickPerRad = 2.f;
constexpr float kMaxYawStick = 1.f;
// the pilot keeps the nose on where the vehicle goes, so the forward-facing sensor sees the way
constexpr float kCourseYawStickPerRad = 2.f;
constexpr float kMaxCourseYawStick = 1.f;
// slower, the direction of travel is noise
constexpr float kCourseSpeedMS = 0.5f;
// stopped this long, the pilot turns to look for a way
constexpr std::chrono::milliseconds kHeldTime{1000};
constexpr float kSearchYawStick = 0.5f;
// outside the stick deadzone, so a turn finishes
constexpr float kMinTurnYawStick = 0.15f;
// for callbacks already running when they are unsubscribed
constexpr std::chrono::milliseconds kCallbackGrace{500};

float wrap_pi(float angle)
{
	while (angle > M_PI) { angle -= 2.f * M_PI; }

	while (angle < -M_PI) { angle += 2.f * M_PI; }

	return angle;
}

class AutopilotTesterCollisionPrevention : public AutopilotTester
{
public:
	void load_world()
	{
		auto params = getParams();
		obstacle_sim::WorldConfig config{};
		config.type = static_cast<obstacle_sim::WorldType>(params->get_param_int("SIH_WLD_TYPE").second);
		config.seed = params->get_param_int("SIH_WLD_SEED").second;
		config.cell_size = params->get_param_float("SIH_WLD_SPACING").second;
		config.density = params->get_param_float("SIH_WLD_DENSITY").second;
		config.clear_radius = params->get_param_float("SIH_WLD_CLEAR").second;
		REQUIRE(config.type == obstacle_sim::WorldType::Trees);
		_world.configure(config);

		// SIH's local frame, which the world is laid out in
		const double lat0 = params->get_param_float("SIH_LOC_LAT0").second;
		const double lon0 = params->get_param_float("SIH_LOC_LON0").second;
		_ground_altitude_m = params->get_param_float("SIH_LOC_H0").second;
		_ct = std::make_unique<CoordinateTransformation>(CoordinateTransformation::GlobalCoordinate{lat0, lon0});

		_cp_dist_m = params->get_param_float("CP_DIST").second;
		REQUIRE(_cp_dist_m > 0.f);
		_half_fov_rad = 0.5f * params->get_param_float("SIH_RIMG_HFOV").second * M_PI / 180.f;
	}

	void start_sampling()
	{
		CHECK(getTelemetry()->set_rate_ground_truth(50) == Telemetry::Result::Success);
		// the pilot steers from these
		CHECK(getTelemetry()->set_rate_attitude_euler(50) == Telemetry::Result::Success);
		CHECK(getTelemetry()->set_rate_position_velocity_ned(50) == Telemetry::Result::Success);
		_ground_truth_handle = getTelemetry()->subscribe_ground_truth([this](Telemetry::GroundTruth g) {
			const auto local = _ct->local_from_global({g.latitude_deg, g.longitude_deg});
			const float north = local.north_m;
			const float east = local.east_m;
			const float down = _ground_altitude_m - g.absolute_altitude_m;
			const float yaw = getTelemetry()->attitude_euler().yaw_deg * M_PI / 180.f;
			obstacle_sim::Tree tree{};
			const float distance = _world.nearestTrunk(north, east, down, kTrunkSearchRadiusM, &tree);

			std::lock_guard<std::mutex> lock(_mutex);
			_north = north;
			_east = east;

			if (_sampling) {
				_samples++;
				track_seen(north, east, yaw);

				if (distance < _min_distance_m) {
					_min_distance_m = distance;
					_closest_tree = tree;
					_closest_north = north;
					_closest_east = east;
				}

				_max_distance_from_home_m = std::max(_max_distance_from_home_m, std::hypot(north, east));
				_max_distance_from_start_m = std::max(_max_distance_from_start_m, std::hypot(north - _start_north, east - _start_east));
			}
		});
	}

	// the callback holds this, so it goes before the tester does
	void stop_sampling()
	{
		getTelemetry()->unsubscribe_ground_truth(_ground_truth_handle);
		std::this_thread::sleep_for(kCallbackGrace);
	}

	// with the mutex held: counts the samples each trunk is in view, and the closest approach to
	// trunks already seen
	void track_seen(float north, float east, float yaw)
	{
		obstacle_sim::Tree trees[kMaxTreesInView];
		const int count = _world.treesNear(north, east, kMapReachM + 1.f, trees, kMaxTreesInView);

		for (int i = 0; i < count; i++) {
			const obstacle_sim::Tree &tree = trees[i];
			const std::pair<long, long> key{std::lround(tree.north * 100.f), std::lround(tree.east * 100.f)};
			const float distance = std::hypot(tree.north - north, tree.east - east) - tree.radius;

			if (_seen_samples[key] >= kSeenSamples && distance < _min_seen_distance_m) {
				_min_seen_distance_m = distance;
				_closest_seen_tree = tree;
			}

			const float bearing = std::atan2(tree.east - east, tree.north - north);

			if (distance + tree.radius > kMapReachM) {
				_seen_samples[key] = 0;

			} else if (std::fabs(wrap_pi(bearing - yaw)) < _half_fov_rad && distance < kSeenRangeM && distance > _cp_dist_m) {
				_seen_samples[key]++;
			}
		}
	}

	void set_sampling(bool on)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_sampling = on;
	}

	void sticks(float pitch, float roll, float throttle, float yaw, std::chrono::milliseconds duration)
	{
		for (auto t = 0ms; t < duration; t += kStickPeriod) {
			CHECK(getManualControl()->set_manual_control_input(pitch, roll, throttle, yaw) == ManualControl::Result::Success);
			sleep_for(kStickPeriod);
		}
	}

	void take_off_in_position_mode()
	{
		// the stick stream has to be up before Position mode and arming are accepted
		sticks(0.f, 0.f, 0.5f, 0.f, 1s);
		REQUIRE(getManualControl()->start_position_control() == ManualControl::Result::Success);
		sticks(0.f, 0.f, 0.5f, 0.f, 1s);
		arm();

		for (auto t = 0ms; t < 20s; t += kStickPeriod) {
			if (getTelemetry()->position_velocity_ned().position.down_m < -kFlightAltitudeM) {
				break;
			}

			sticks(0.f, 0.f, 1.f, 0.f, kStickPeriod);
		}

		sticks(0.f, 0.f, 0.5f, 0.f, 2s);
		REQUIRE(getTelemetry()->position_velocity_ned().position.down_m < -0.8f * kFlightAltitudeM);
	}

	// the trunk the vehicle is pointed at, the one nearest to where it hovers
	obstacle_sim::Tree target_trunk()
	{
		float north;
		float east;
		{
			std::lock_guard<std::mutex> lock(_mutex);
			north = _north;
			east = _east;
		}

		obstacle_sim::Tree tree{};
		REQUIRE(std::isfinite(_world.nearestTrunk(north, east, -kFlightAltitudeM, kTrunkSearchRadiusM, &tree)));
		std::cout << time_str() << "Target trunk at " << tree.north << ", " << tree.east << " m, radius " << tree.radius
			  << " m, " << std::hypot(tree.north - north, tree.east - east) << " m away" << std::endl;
		return tree;
	}

	float bearing_to(const obstacle_sim::Tree &tree)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return std::atan2(tree.east - _east, tree.north - _north);
	}

	float yaw_stick_to(float bearing_rad)
	{
		const float error = wrap_pi(bearing_rad - getTelemetry()->attitude_euler().yaw_deg * M_PI / 180.f);
		const float stick = std::clamp(kYawStickPerRad * error, -kMaxYawStick, kMaxYawStick);
		return std::copysign(std::max(std::fabs(stick), kMinTurnYawStick), stick);
	}

	// yaw stick until the nose points along bearing
	void turn_to(float bearing_rad)
	{
		constexpr float kAlignedRad = 2.f * M_PI / 180.f;

		for (auto t = 0ms; t < 30s; t += kStickPeriod) {
			const float error = wrap_pi(bearing_rad - getTelemetry()->attitude_euler().yaw_deg * M_PI / 180.f);

			if (std::fabs(error) < kAlignedRad) {
				sticks(0.f, 0.f, 0.5f, 0.f, 1s);
				return;
			}

			sticks(0.f, 0.f, 0.5f, yaw_stick_to(bearing_rad), kStickPeriod);
		}

		FAIL("did not turn to the trunk");
	}

	// full forward stick, the nose turned to where the vehicle goes, and turning away once stopped
	void fly_forward(std::chrono::milliseconds duration)
	{
		auto held_for = 0ms;

		for (auto t = 0ms; t < duration; t += kStickPeriod) {
			const auto velocity = getTelemetry()->position_velocity_ned().velocity;
			float yaw = 0.f;

			if (std::hypot(velocity.north_m_s, velocity.east_m_s) > kCourseSpeedMS) {
				held_for = 0ms;
				const float course = std::atan2(velocity.east_m_s, velocity.north_m_s);
				const float error = wrap_pi(course - getTelemetry()->attitude_euler().yaw_deg * M_PI / 180.f);
				yaw = std::clamp(kCourseYawStickPerRad * error, -kMaxCourseYawStick, kMaxCourseYawStick);

			} else {
				held_for += kStickPeriod;
				yaw = (held_for > kHeldTime) ? kSearchYawStick : 0.f;
			}

			sticks(1.f, 0.f, 0.5f, yaw, kStickPeriod);
		}
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

	void store_start()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_start_north = _north;
		_start_east = _east;
	}

	float max_distance_from_start()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _max_distance_from_start_m;
	}

	void check_clearance(float min_clearance_m)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		std::cout << time_str() << "Closest approach " << _min_distance_m << " m to the trunk at " << _closest_tree.north
			  << ", " << _closest_tree.east << " m, from " << _closest_north << ", " << _closest_east << " m, over "
			  << _samples << " samples; CP_DIST " << _cp_dist_m << " m" << std::endl;
		std::cout << time_str() << "Closest approach to a trunk the sensor had seen " << _min_seen_distance_m << " m, the trunk at "
			  << _closest_seen_tree.north << ", " << _closest_seen_tree.east << " m" << std::endl;
		REQUIRE(_samples > 100);
		CHECK(_min_seen_distance_m > min_clearance_m);
		CHECK(_min_distance_m > kNoContactClearanceM);
		CHECK(_min_distance_m < _cp_dist_m + kEngagedMarginM);
	}

	float cp_dist() const { return _cp_dist_m; }

	float max_distance_from_home()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _max_distance_from_home_m;
	}

private:
	obstacle_sim::World _world{};
	std::unique_ptr<CoordinateTransformation> _ct{};
	float _ground_altitude_m{NAN};
	float _cp_dist_m{NAN};

	std::mutex _mutex;
	Telemetry::GroundTruthHandle _ground_truth_handle{};
	bool _sampling{false};
	unsigned _samples{0};
	float _north{NAN};
	float _east{NAN};
	float _min_distance_m{INFINITY};
	float _min_seen_distance_m{INFINITY};
	obstacle_sim::Tree _closest_seen_tree{};
	std::map<std::pair<long, long>, unsigned> _seen_samples{};
	float _half_fov_rad{NAN};
	obstacle_sim::Tree _closest_tree{};
	float _closest_north{NAN};
	float _closest_east{NAN};
	float _max_distance_from_home_m{0.f};
	float _start_north{NAN};
	float _start_east{NAN};
	float _max_distance_from_start_m{0.f};
};

} // namespace

TEST_CASE("Collision Prevention keeps the vehicle off the trunks", "[collision_prevention]")
{
	AutopilotTesterCollisionPrevention tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.load_world();
	tester.store_home();
	tester.start_sampling();

	tester.take_off_in_position_mode();

	const obstacle_sim::Tree trunk = tester.target_trunk();
	const float bearing = tester.bearing_to(trunk);
	tester.turn_to(bearing);

	// full forward stick straight at the trunk, then on through the trees behind it
	tester.store_start();
	tester.set_sampling(true);
	tester.fly_forward(90s);
	tester.set_sampling(false);

	const float travel = tester.max_distance_from_start();
	std::cout << time_str() << "Reached " << travel << " m from the start, " << tester.max_distance_from_home()
		  << " m from home" << std::endl;

	tester.land();
	tester.stop_sampling();

	tester.check_clearance(tester.cp_dist() - kToleranceM);
	// the pilot got through the trees rather than stopped at the first
	CHECK(travel > kMinTravelM);
}
