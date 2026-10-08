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

#include "ObstacleMap.hpp"

#include <lib/parameters/param.h>
#include <px4_platform_common/log.h>
#include <string.h>

using namespace matrix;
using obstacle_map::ObstacleGrid;

static_assert(range_image::kRangeNoReturn == range_image_s::RANGE_NO_RETURN, "range counts differ from RangeImage.msg");
static_assert(range_image::kRangeMaxCount == range_image_s::RANGE_MAX_COUNT, "range counts differ from RangeImage.msg");
static_assert(range_image::kRangeBelowMin == range_image_s::RANGE_BELOW_MIN, "range counts differ from RangeImage.msg");
static_assert(range_image::kRangeInvalid == range_image_s::RANGE_INVALID, "range counts differ from RangeImage.msg");
static_assert(range_image::kProjectionPinhole == range_image_info_s::PROJECTION_PINHOLE, "enum differs from RangeImageInfo.msg");
static_assert(range_image::kRangeTypeDepth == range_image_info_s::RANGE_TYPE_DEPTH, "enum differs from RangeImageInfo.msg");
static_assert(range_image::kZoneOrderColumnMajor == range_image_info_s::ZONE_ORDER_COLUMN_MAJOR,
	      "enum differs from RangeImageInfo.msg");
static_assert(obstacle_map_status_s::SLICE_SIZE * obstacle_map_status_s::SLICE_SIZE / 4 == sizeof(
		      obstacle_map_status_s::slice), "slice is 2 bits per cell");

// a pose this far past the newest sample is extrapolated with the estimated velocity
static constexpr hrt_abstime kMaxExtrapolation = 50_ms;
static constexpr float kBinWidthDeg = 360.f / (sizeof(obstacle_distance_s::distances) / sizeof(
		obstacle_distance_s::distances[0]));
static constexpr float kMetersToCentimeters = 100.f;

ModuleBase::Descriptor ObstacleMap::desc{task_spawn, custom_command, print_usage};

ObstacleMap::ObstacleMap() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ObstacleMap::~ObstacleMap()
{
	perf_free(_cycle_perf);
	perf_free(_insert_perf);
	perf_free(_sectors_perf);
	perf_free(_sweeps_perf);
}

bool ObstacleMap::init()
{
	if (!_grid.allocate(CONFIG_OBSTACLE_MAP_SIZE_XY, CONFIG_OBSTACLE_MAP_SIZE_Z)) {
		PX4_ERR("map allocation failed");
		return false;
	}

	updateParameters();

	if (!_range_image_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// tiles schedule a run as they arrive, the interval keeps the pose history and the output going
	ScheduleOnInterval(kSchedule);
	return true;
}

void ObstacleMap::updateParameters()
{
	_grid.setVoxelSize(_param_omap_vox_size.get());

	ObstacleGrid::Weights weights{};
	weights.hit = _param_omap_hit.get();
	weights.miss = _param_omap_miss.get();
	weights.occupied = _param_omap_occ_thr.get();
	weights.free = _param_omap_free_thr.get();
	weights.clamp_min = _param_omap_lo_min.get();
	_grid.setWeights(weights);

	// Collision Prevention's vertical distance, so the ground and ceilings it keeps away from
	// vertically are not horizontal obstacles too
	const param_t cp_dist_v = param_find("CP_DIST_V");
	float vertical_margin = 0.f;

	if (cp_dist_v != PARAM_INVALID && param_get(cp_dist_v, &vertical_margin) == PX4_OK) {
		_vertical_margin = fmaxf(vertical_margin, 0.f);
	}
}

void ObstacleMap::Run()
{
	if (should_exit()) {
		_range_image_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		updateParameters();
	}

	updatePose();

	if (_local_position_valid) {
		const float position[3] {_local_position.x, _local_position.y, _local_position.z};
		_grid.recenter(position);
	}

	insertTiles();

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_obstacle_distance >= kObstacleDistanceInterval && mapIsFresh(now)) {
		publishObstacleDistance(now);
		publishClearance(now);
		_last_obstacle_distance = now;
	}

	if (now - _last_status >= kStatusInterval) {
		publishStatus(now);
	}

	perf_end(_cycle_perf);
}

void ObstacleMap::updatePose()
{
	vehicle_local_position_s local_position;

	if (_local_position_sub.update(&local_position)) {
		if (_reset_counters_known) {
			if (local_position.heading_reset_counter != _local_position.heading_reset_counter) {
				// a voxel grid cannot be rotated without losing data
				_grid.clear();
				_position_count = 0;
				_attitude_count = 0;

			} else {
				float delta[3] {};
				bool shifted = false;

				if (local_position.xy_reset_counter != _local_position.xy_reset_counter) {
					delta[0] = local_position.delta_xy[0];
					delta[1] = local_position.delta_xy[1];
					shifted = true;
				}

				if (local_position.z_reset_counter != _local_position.z_reset_counter) {
					delta[2] = local_position.delta_z;
					shifted = true;
				}

				if (shifted) {
					_grid.shiftOrigin(delta);
					// the history is in the old frame
					_position_count = 0;
				}
			}
		}

		_reset_counters_known = true;
		_local_position = local_position;
		_local_position_valid = local_position.xy_valid && local_position.z_valid;

		if (_local_position_valid) {
			_position_newest = (_position_newest + 1) % kHistory;
			_positions[_position_newest] = {local_position.timestamp_sample, Vector3f(local_position.x, local_position.y, local_position.z)};
			_position_count = math::min(_position_count + 1, kHistory);
		}
	}

	vehicle_attitude_s attitude;

	if (_attitude_sub.update(&attitude)) {
		_attitude = Quatf(attitude.q);
		_attitude_valid = true;
		_attitude_newest = (_attitude_newest + 1) % kHistory;
		_attitudes[_attitude_newest] = {attitude.timestamp_sample, _attitude};
		_attitude_count = math::min(_attitude_count + 1, kHistory);
	}
}

bool ObstacleMap::positionAt(hrt_abstime timestamp, Vector3f &position) const
{
	if (_position_count == 0) {
		return false;
	}

	const PositionSample &newest = _positions[_position_newest];

	if (timestamp >= newest.timestamp) {
		const float dt = math::min(timestamp - newest.timestamp, kMaxExtrapolation) * 1e-6f;
		const Vector3f velocity(_local_position.vx, _local_position.vy, _local_position.vz);
		position = newest.position + (velocity.isAllFinite() ? velocity *dt : Vector3f());
		return true;
	}

	for (int i = 1; i < _position_count; i++) {
		const PositionSample &newer = _positions[(_position_newest - i + 1 + kHistory) % kHistory];
		const PositionSample &older = _positions[(_position_newest - i + kHistory) % kHistory];

		if (older.timestamp <= timestamp) {
			const float span = (newer.timestamp - older.timestamp) * 1e-6f;
			const float a = (span > 0.f) ? (timestamp - older.timestamp) * 1e-6f / span : 1.f;
			position = older.position + (newer.position - older.position) * a;
			return true;
		}
	}

	// older than the history, the oldest sample is the best there is
	position = _positions[(_position_newest - _position_count + 1 + kHistory) % kHistory].position;
	return true;
}

bool ObstacleMap::attitudeAt(hrt_abstime timestamp, Quatf &attitude) const
{
	if (_attitude_count == 0) {
		return false;
	}

	const AttitudeSample &newest = _attitudes[_attitude_newest];

	if (timestamp >= newest.timestamp) {
		attitude = newest.attitude;
		return true;
	}

	for (int i = 1; i < _attitude_count; i++) {
		const AttitudeSample &newer = _attitudes[(_attitude_newest - i + 1 + kHistory) % kHistory];
		const AttitudeSample &older = _attitudes[(_attitude_newest - i + kHistory) % kHistory];

		if (older.timestamp <= timestamp) {
			const float span = (newer.timestamp - older.timestamp) * 1e-6f;
			const float a = (span > 0.f) ? (timestamp - older.timestamp) * 1e-6f / span : 1.f;
			// normalised linear interpolation, samples are milliseconds apart
			const Quatf to = (older.attitude.dot(newer.attitude) < 0.f) ? Quatf(-newer.attitude) : newer.attitude;
			attitude = Quatf(older.attitude * (1.f - a) + to * a).normalized();
			return true;
		}
	}

	attitude = _attitudes[(_attitude_newest - _attitude_count + 1 + kHistory) % kHistory].attitude;
	return true;
}

void ObstacleMap::insertTiles()
{
	range_image_info_s info;

	if (_range_image_info_sub.update(&info)) {
		_info = info;
		_info_valid = info.num_rows > 0 && info.num_cols > 0 && info.range_lsb_mm > 0;
	}

	range_image_s tile;

	while (_range_image_sub.update(&tile)) {
		Vector3f position;
		Quatf attitude;

		if (!_info_valid || tile.device_id != _info.device_id || tile.config_id != _info.config_id
		    || !_local_position_valid || !positionAt(tile.timestamp_sample, position)
		    || !attitudeAt(tile.timestamp_sample, attitude)) {
			_tiles_dropped++;
			continue;
		}

		range_image::Geometry geometry{};
		geometry.num_rows = _info.num_rows;
		geometry.num_cols = _info.num_cols;
		geometry.projection = _info.projection;
		geometry.range_type = _info.range_type;
		geometry.zone_order = _info.zone_order;
		geometry.x_start = _info.x_start;
		geometry.x_step = _info.x_step;
		geometry.y_start = _info.y_start;
		geometry.y_step = _info.y_step;
		geometry.row_angle = _info.row_angle;
		geometry.row_angle_count = math::min(_info.row_angle_count, (uint8_t)(sizeof(_info.row_angle) / sizeof(_info.row_angle[0])));

		// a sensor that does not know its mounting looks forward from the body origin
		Quatf body_sensor(_info.q_body_sensor);

		if (body_sensor.norm() < FLT_EPSILON) {
			body_sensor = Quatf();
		}

		const Dcmf local_sensor(attitude * body_sensor.normalized());
		const Vector3f origin = position + attitude.rotateVector(Vector3f(_info.position_body));

		obstacle_map::SensorPose pose{};
		origin.copyTo(pose.origin);

		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				pose.rotation[row * 3 + col] = local_sensor(row, col);
			}
		}

		obstacle_map::Tile ranges{};
		ranges.first_zone = tile.first_zone;
		ranges.ranges = tile.ranges;
		ranges.num_ranges = math::min((int)tile.num_ranges, (int)(sizeof(tile.ranges) / sizeof(tile.ranges[0])));
		ranges.lsb = _info.range_lsb_mm * 1e-3f;
		ranges.range_min = _info.range_min;
		ranges.range_max = _info.range_max;

		perf_begin(_insert_perf);
		_rays_inserted += obstacle_map::insertTile(_grid, geometry, pose, ranges);
		perf_end(_insert_perf);

		_last_tile = tile.timestamp;
	}
}

bool ObstacleMap::mapIsFresh(hrt_abstime now) const
{
	// without fresh range images the map goes stale with the world, so Collision Prevention
	// should see the sensor drop out rather than a map that no longer updates
	return _local_position_valid && _attitude_valid && _last_tile != 0 && now - _last_tile <= kTileTimeout;
}

void ObstacleMap::publishObstacleDistance(hrt_abstime now)
{
	const float position[3] {_local_position.x, _local_position.y, _local_position.z};
	// the window reaches half its size either side, less the voxel the vehicle sits in
	const float max_range = (_grid.sizeXY() / 2 - 1) * _grid.voxelSize();
	const uint16_t max_range_cm = (uint16_t)(max_range * kMetersToCentimeters);

	float distance[kBins];
	perf_begin(_sectors_perf);
	const obstacle_map::Band band = obstacle_map::bandAround(_grid, position, _param_omap_veh_rad.get(),
					0.5f * _param_omap_veh_hgt.get(), _vertical_margin);
	obstacle_map::sectorDistances(_grid, position, Eulerf(_attitude).psi(), band, max_range, kBins, distance);
	perf_end(_sectors_perf);

	obstacle_distance_s obstacle_distance{};
	obstacle_distance.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	obstacle_distance.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	obstacle_distance.increment = kBinWidthDeg;
	obstacle_distance.angle_offset = 0.f;
	obstacle_distance.min_distance = 0;
	obstacle_distance.max_distance = max_range_cm;

	for (int i = 0; i < kBins; i++) {
		if (__builtin_isnan(distance[i])) {
			obstacle_distance.distances[i] = UINT16_MAX;

		} else if (__builtin_isinf(distance[i])) {
			obstacle_distance.distances[i] = max_range_cm + 1;

		} else {
			// 0 would read as no obstacle to Collision Prevention
			obstacle_distance.distances[i] = math::constrain((uint16_t)lroundf(distance[i] * kMetersToCentimeters), (uint16_t)1,
							 max_range_cm);
		}
	}

	obstacle_distance.timestamp = hrt_absolute_time();
	_obstacle_distance_pub.publish(obstacle_distance);
}

void ObstacleMap::publishClearance(hrt_abstime now)
{
	const float position[3] {_local_position.x, _local_position.y, _local_position.z};
	const float radius = _param_omap_veh_rad.get();
	const obstacle_map::Band body = obstacle_map::bandAround(_grid, position, radius, 0.5f * _param_omap_veh_hgt.get(), 0.f);
	const float max_distance = (_grid.sizeXY() / 2 - 1) * _grid.voxelSize();

	obstacle_clearance_s clearance{};
	clearance.timestamp_sample = _local_position.timestamp_sample;
	clearance.body_radius = radius;
	clearance.max_distance = max_distance;

	Vector3f directions[obstacle_clearance_s::SWEEP_COUNT];
	directions[obstacle_clearance_s::SWEEP_UP] = Vector3f(0.f, 0.f, -1.f);
	directions[obstacle_clearance_s::SWEEP_DOWN] = Vector3f(0.f, 0.f, 1.f);
	directions[obstacle_clearance_s::SWEEP_SETPOINT] = Vector3f(NAN, NAN, NAN);
	directions[obstacle_clearance_s::SWEEP_VELOCITY] = (_local_position.v_xy_valid && _local_position.v_z_valid)
			? Vector3f(_local_position.vx, _local_position.vy, _local_position.vz) : Vector3f(NAN, NAN, NAN);

	trajectory_setpoint_s setpoint;

	if (_trajectory_setpoint_sub.copy(&setpoint) && now - setpoint.timestamp < kTileTimeout) {
		directions[obstacle_clearance_s::SWEEP_SETPOINT] = Vector3f(setpoint.velocity);
	}

	perf_begin(_sweeps_perf);

	for (int i = 0; i < obstacle_clearance_s::SWEEP_COUNT; i++) {
		clearance.contact[i] = NAN;
		clearance.observed[i] = NAN;
		const Vector3f &direction = directions[i];

		// without motion there is no direction to sweep along
		if (!direction.isAllFinite() || !direction.longerThan(kMinSweepSpeed)) {
			continue;
		}

		const Vector3f unit = direction.normalized();
		const float unit_direction[3] {unit(0), unit(1), unit(2)};
		// A ceiling or the ground seen at a grazing angle is mapped in patches, as the rays to its
		// far side miss the voxels they cross under it. Straight up and down the body is swept wider
		// than it is, so a hole of that size is not a way through.
		const bool vertical = (i == obstacle_clearance_s::SWEEP_UP || i == obstacle_clearance_s::SWEEP_DOWN);
		const float sweep_radius = vertical ? radius + kVerticalSweepMargin * _grid.voxelSize() : radius;
		const obstacle_map::Sweep sweep = obstacle_map::sweepBody(_grid, position, unit_direction, sweep_radius, body,
						  max_distance);
		clearance.direction_north[i] = unit(0);
		clearance.direction_east[i] = unit(1);
		clearance.direction_down[i] = unit(2);
		clearance.contact[i] = sweep.contact;
		clearance.observed[i] = sweep.observed;
		clearance.contact_face[i] = (sweep.face == obstacle_map::Sweep::Face::Top) ? obstacle_clearance_s::FACE_TOP
					    : (sweep.face == obstacle_map::Sweep::Face::Bottom) ? obstacle_clearance_s::FACE_BOTTOM
					    : obstacle_clearance_s::FACE_SIDE;
	}

	perf_end(_sweeps_perf);

	clearance.timestamp = hrt_absolute_time();
	_clearance_pub.publish(clearance);
}

void ObstacleMap::publishStatus(hrt_abstime now)
{
	_last_status = now;

	obstacle_map_status_s status{};
	status.voxel_size = _grid.voxelSize();
	status.grid_size_xy = _grid.sizeXY();
	status.grid_size_z = _grid.sizeZ();
	memcpy(status.center_voxel, _grid.center(), sizeof(status.center_voxel));
	obstacle_map::countStates(_grid, status.occupied_count, status.free_count);
	status.rays_inserted = _rays_inserted;

	if (_local_position_valid) {
		const float position[3] {_local_position.x, _local_position.y, _local_position.z};
		status.slice_down_voxel = _grid.voxelIndex(position[2]);

		const int half = obstacle_map_status_s::SLICE_SIZE / 2;

		for (int n = 0; n < obstacle_map_status_s::SLICE_SIZE; n++) {
			for (int e = 0; e < obstacle_map_status_s::SLICE_SIZE; e++) {
				const ObstacleGrid::State state = _grid.state(_grid.center()[0] - half + n, _grid.center()[1] - half + e,
								  status.slice_down_voxel);
				const uint8_t cell = (state == ObstacleGrid::State::Occupied) ? obstacle_map_status_s::CELL_OCCUPIED
						     : (state == ObstacleGrid::State::Free) ? obstacle_map_status_s::CELL_FREE : obstacle_map_status_s::CELL_UNKNOWN;
				const int bit = 2 * (n * obstacle_map_status_s::SLICE_SIZE + e);
				status.slice[bit / 8] |= cell << (bit % 8);
			}
		}
	}

	status.timestamp = hrt_absolute_time();
	_status_pub.publish(status);
}

int ObstacleMap::task_spawn(int argc, char *argv[])
{
	ObstacleMap *instance = new ObstacleMap();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ObstacleMap::print_status()
{
	uint32_t occupied = 0;
	uint32_t free = 0;
	obstacle_map::countStates(_grid, occupied, free);
	PX4_INFO("grid %d x %d x %d at %.2f m, %" PRIu32 " occupied, %" PRIu32 " free", _grid.sizeXY(), _grid.sizeXY(),
		 _grid.sizeZ(), (double)_grid.voxelSize(), occupied, free);
	PX4_INFO("sensor %s, %" PRIu32 " rays inserted, %" PRIu32 " tiles dropped", _info_valid ? "described" : "not described",
		 _rays_inserted, _tiles_dropped);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_insert_perf);
	perf_print_counter(_sectors_perf);
	perf_print_counter(_sweeps_perf);
	return 0;
}

int ObstacleMap::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ObstacleMap::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Persistent 3D occupancy map in the local frame, built from range_image tiles.

The map is a rolling grid of 4-bit log-odds voxels centred on the vehicle. Each return walks
its ray through the grid: voxels it passes become more likely free, the voxel it ends in more
likely occupied. Unobserved voxels keep their state, so obstacles stay mapped after the sensor
looks away.

The nearest occupied voxel in each 5 degree sector, within a band of OMAP_VEH_HGT plus
CP_DIST_V above and below the vehicle, is published as obstacle_distance for Collision
Prevention. Sectors never observed are reported as unknown. The band stops short of a surface
directly above or below the vehicle, so the ground it flies over is not an obstacle.

The vehicle's body is swept up, down, along the velocity setpoint and along the velocity, and
how far it can move before touching an occupied voxel is published as obstacle_clearance.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("obstacle_map", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int obstacle_map_main(int argc, char *argv[])
{
	return ModuleBase::main(ObstacleMap::desc, argc, argv);
}
