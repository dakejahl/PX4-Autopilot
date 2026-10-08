/****************************************************************************
 *
 *   Copyright (c) 2018-2024 PX4 Development Team. All rights reserved.
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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include "CollisionPrevention.hpp"
#include "ObstacleMath.hpp"
#include <lib/mathlib/math/TrajMath.hpp>
#include <px4_platform_common/events.h>
#include <math.h>


using namespace matrix;

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(BIN_SIZE >= 5, "BIN_SIZE must be at least 5");
	static_assert(360 % BIN_SIZE == 0, "BIN_SIZE must divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = BIN_SIZE;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;

	for (uint32_t i = 0 ; i < BIN_COUNT; i++) {
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
	}
}

hrt_abstime CollisionPrevention::getTime()
{
	return hrt_absolute_time();
}

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
	return hrt_absolute_time() - *ptr;
}

bool CollisionPrevention::is_active()
{
	bool activated = _param_cp_dist.get() > 0;

	if (activated && !_was_active) {
		_time_activated = getTime();
	}

	_was_active = activated;
	return activated;
}

void CollisionPrevention::modifySetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			_vehicle_attitude = Quatf(vehicle_attitude.q);
			_vehicle_yaw = Eulerf(_vehicle_attitude).psi();
		}
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s local_position;

		if (_vehicle_local_position_sub.copy(&local_position)) {
			_velocity_estimate = local_position.v_xy_valid ? Vector2f(local_position.vx, local_position.vy) : Vector2f(NAN, NAN);
			_vertical_velocity_estimate = local_position.v_z_valid ? local_position.vz : NAN;
		}
	}

	//calculate movement constraints based on range data
	const Vector2f original_setpoint = setpoint_accel;
	_updateObstacleMap();
	_updateObstacleData();
	_updateClearance(getTime());
	_calculateConstrainedSetpoint(setpoint_accel, setpoint_vel);

	// publish constraints
	collision_constraints_s	constraints{};
	original_setpoint.copyTo(constraints.original_setpoint);
	setpoint_accel.copyTo(constraints.adapted_setpoint);
	constraints.timestamp = getTime();
	_constraints_pub.publish(constraints);
}

void CollisionPrevention::_updateObstacleMap()
{
	// add distance sensor data
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			// consider only instances with valid data and orientations useful for collision prevention
			if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				// update message description
				_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
				_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
									(uint16_t)(distance_sensor.max_distance * 100.0f));
				_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
									(uint16_t)(distance_sensor.min_distance * 100.0f));

				_addDistanceSensorData(distance_sensor, _vehicle_attitude);
			}
		}
	}

	// add obstacle distance data
	if (_sub_obstacle_distance.update()) {
		const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

		// Update map with obstacle data if the data is not stale
		if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US && obstacle_distance.increment > 0.f) {
			//update message description
			_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, _vehicle_yaw);
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_fused_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_updateObstacleData()
{
	_obstacle_data_present = false;

	for (int i = 0; i < BIN_COUNT; i++) {
		// if the data is stale, reset the bin
		if (getTime() - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
			_obstacle_map_body_frame.distances[i] = UINT16_MAX;
		}

		const uint16_t bin_distance = _obstacle_map_body_frame.distances[i];

		// check if there is avaliable data and the data of the map is not stale
		if (bin_distance < UINT16_MAX
		    && (getTime() - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
			_obstacle_data_present = true;
		}
	}
}

void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint_accel, const Vector2f &setpoint_vel)
{
	using namespace collision_prevention;

	const hrt_abstime now = getTime();
	_min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, _param_cp_dist.get());

	if (!_obstacle_data_present) {
		// allow no movement
		setpoint_accel.setZero();

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {
			if ((now - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US &&
			    getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			PX4_WARN("No obstacle data, not moving...");
			_last_timeout_warning = now;
		}

		return;
	}

	// the map is in the heading frame
	const float cos_yaw = cosf(_vehicle_yaw);
	const float sin_yaw = sinf(_vehicle_yaw);
	const auto to_heading = [&](const Vector2f & v) { return Vector2f(cos_yaw * v(0) + sin_yaw * v(1), -sin_yaw * v(0) + cos_yaw * v(1)); };
	const auto to_local = [&](const Vector2f & v) { return Vector2f(cos_yaw * v(0) - sin_yaw * v(1), sin_yaw * v(0) + cos_yaw * v(1)); };

	const Vector2f velocity = to_heading(setpoint_vel);
	// without an estimate the setpoint stands in for it
	const Vector2f velocity_estimate = _velocity_estimate.isAllFinite() ? to_heading(_velocity_estimate) : velocity;

	PolarObstacles &obstacles = _obstacles;
	_polarObstacles(obstacles);

	// an obstacle measured a while ago is closer by what the vehicle has flown towards it since
	for (int k = 0; k < kSectors; k++) {
		if (PX4_ISFINITE(obstacles.distance[k])) {
			const float age = math::constrain((now - _data_timestamps[k]) * 1e-6f, 0.f, RANGE_STREAM_TIMEOUT_US * 1e-6f);
			const float closing = math::max(velocity_estimate.dot(sectorDirection(k)), 0.f);
			obstacles.distance[k] = math::max(obstacles.distance[k] - closing * age, 0.f);
		}
	}

	PathLimit path_limits[MAX_PATH_LIMITS];

	for (int i = 0; i < _path_limit_count; i++) {
		path_limits[i] = {to_heading(_path_limits[i].direction), _path_limits[i].max_speed};
	}

	LimitConfig config{};
	config.distance = _min_dist_to_keep;
	config.max_acceleration = _param_mpc_acc_hor.get();
	config.max_jerk = _param_mpc_jerk_max.get();
	config.gain = _param_mpc_xy_vel_p_acc.get();
	config.delay = _param_cp_delay.get();
	config.push_time = PUSH_TIME;

	setpoint_accel = to_local(limitAcceleration(to_heading(setpoint_accel), velocity, velocity_estimate, obstacles, config,
				  _limit_workspace, path_limits, _path_limit_count));
}

void CollisionPrevention::_updateClearance(hrt_abstime now)
{
	_speed_limit_up = INFINITY;
	_speed_limit_down = INFINITY;
	_path_limit_count = 0;

	_sub_obstacle_clearance.update();
	const obstacle_clearance_s &clearance = _sub_obstacle_clearance.get();

	// published after now was taken counts as fresh
	if (clearance.timestamp == 0 || (now > clearance.timestamp && now - clearance.timestamp > RANGE_STREAM_TIMEOUT_US)) {
		return;
	}

	const float gap_vertical = math::max(_param_cp_dist_v.get(), 0.f);
	const float gap_horizontal = math::max(_param_cp_dist.get() - clearance.body_radius, 0.f);
	const float down_speed = PX4_ISFINITE(_vertical_velocity_estimate) ? _vertical_velocity_estimate : 0.f;
	const float land_speed = _param_mpc_land_speed.get();

	// as fast as the vehicle can still stop within room, after flying on for the delay
	const auto max_speed = [&](float room, float speed, float acceleration) {
		const float available = room - math::max(speed, 0.f) * _param_cp_delay.get();
		return (available > 0.f) ? math::trajectory::computeMaxSpeedFromDistance(_param_mpc_jerk_max.get(), acceleration,
				available, 0.f) : 0.f;
	};

	// Straight up and down, unseen space counts as an obstacle just past the gap, as it does
	// horizontally. Descending never slows below the landing speed, or the vehicle could not land.
	const auto room = [&](int sweep) {
		float r = PX4_ISFINITE(clearance.observed[sweep]) ? clearance.observed[sweep] + gap_vertical : INFINITY;

		if (PX4_ISFINITE(clearance.contact[sweep])) {
			r = math::min(r, clearance.contact[sweep] - gap_vertical);
		}

		return r;
	};

	const float room_up = room(obstacle_clearance_s::SWEEP_UP);
	const float room_down = room(obstacle_clearance_s::SWEEP_DOWN);

	if (PX4_ISFINITE(room_up)) {
		_speed_limit_up = max_speed(room_up, -down_speed, _param_mpc_acc_down_max.get());
	}

	if (PX4_ISFINITE(room_down)) {
		_speed_limit_down = math::max(max_speed(room_down, down_speed, _param_mpc_acc_up_max.get()), land_speed);
	}

	// Along the path, contacts only: whether unseen space ahead is passable is the sectors' call.
	// Each limits the motion towards the face it is on.
	static constexpr int PATH_SWEEPS[] {obstacle_clearance_s::SWEEP_SETPOINT, obstacle_clearance_s::SWEEP_VELOCITY};

	for (int sweep : PATH_SWEEPS) {
		const float contact = clearance.contact[sweep];

		if (!PX4_ISFINITE(contact)) {
			continue;
		}

		const Vector3f direction(clearance.direction_north[sweep], clearance.direction_east[sweep],
					 clearance.direction_down[sweep]);

		switch (clearance.contact_face[sweep]) {
		case obstacle_clearance_s::FACE_TOP:
			if (direction(2) < 0.f) {
				_speed_limit_up = math::min(_speed_limit_up, max_speed(-direction(2) * contact - gap_vertical, -down_speed,
							    _param_mpc_acc_down_max.get()));
			}

			break;

		case obstacle_clearance_s::FACE_BOTTOM:
			if (direction(2) > 0.f) {
				_speed_limit_down = math::min(_speed_limit_down, math::max(max_speed(direction(2) * contact - gap_vertical,
							      down_speed, _param_mpc_acc_up_max.get()), land_speed));
			}

			break;

		default: {
				const Vector2f horizontal = direction.xy();

				if (horizontal.longerThan(FLT_EPSILON) && _path_limit_count < MAX_PATH_LIMITS) {
					const Vector2f u = horizontal.normalized();
					const float speed = _velocity_estimate.isAllFinite() ? _velocity_estimate.dot(u) : 0.f;
					_path_limits[_path_limit_count++] = {u, max_speed(horizontal.norm() * contact - gap_horizontal, speed, _param_mpc_acc_hor.get())};
				}

				break;
			}
		}
	}
}

bool CollisionPrevention::verticalSpeedLimits(float &up, float &down) const
{
	if (!_was_active || (!PX4_ISFINITE(_speed_limit_up) && !PX4_ISFINITE(_speed_limit_down))) {
		return false;
	}

	up = _speed_limit_up;
	down = _speed_limit_down;
	return true;
}

void CollisionPrevention::_polarObstacles(collision_prevention::PolarObstacles &obstacles) const
{
	static_assert(BIN_COUNT == collision_prevention::kSectors, "one sector per obstacle_distance bin");

	for (int i = 0; i < BIN_COUNT; i++) {
		const uint16_t distance = _obstacle_map_body_frame.distances[i];
		obstacles.range[i] = _data_maxranges[i] * 0.01f;

		if (distance == UINT16_MAX) {
			obstacles.distance[i] = NAN;

		} else if (distance >= _data_maxranges[i]) {
			// out of range: free as far as the sensor sees
			obstacles.distance[i] = INFINITY;

		} else {
			obstacles.distance[i] = distance * 0.01f;
		}
	}
}

// TODO this gives false output if the offset is not a multiple of the resolution. to be fixed...
void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw)
{

	float vehicle_orientation_deg = math::degrees(vehicle_yaw);


	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; (j < 360 / obstacle.increment) && (j < BIN_COUNT); j++) {
				float bin_lower_angle = ObstacleMath::get_lower_bound_angle(i, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float bin_upper_angle = ObstacleMath::get_lower_bound_angle(i + 1, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float msg_lower_angle = ObstacleMath::get_lower_bound_angle(j, obstacle.increment,
							obstacle.angle_offset - vehicle_orientation_deg);
				float msg_upper_angle = ObstacleMath::get_lower_bound_angle(j + 1, obstacle.increment,
							obstacle.angle_offset - vehicle_orientation_deg);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
				    (msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
				    (msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
				    (msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {
						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
						}
					}
				}

			}
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; j < 360 / obstacle.increment; j++) {
				float bin_lower_angle = ObstacleMath::get_lower_bound_angle(i, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float bin_upper_angle = ObstacleMath::get_lower_bound_angle(i + 1, _obstacle_map_body_frame.increment,
							_obstacle_map_body_frame.angle_offset);
				float msg_lower_angle = ObstacleMath::get_lower_bound_angle(j, obstacle.increment, obstacle.angle_offset);
				float msg_upper_angle = ObstacleMath::get_lower_bound_angle(j + 1, obstacle.increment, obstacle.angle_offset);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
				    (msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
				    (msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
				    (msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {

						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
						}
					}
				}

			}
		}

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Obstacle message received in unsupported frame %i\t",
				     obstacle.frame);
		events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
				      "Obstacle message received in unsupported frame {1}", obstacle.frame);
	}
}

bool
CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
	//use data from this sensor if:
	//1. this sensor data is in range, the bin contains already valid data and this data is coming from the same or less range sensor
	//2. this sensor data is in range, and the last reading was out of range
	//3. this sensor data is out of range, the last reading was as well and this is the sensor with longest range
	//4. this sensor data is out of range, the last reading was valid and coming from the same sensor

	uint16_t sensor_range_cm = static_cast<uint16_t>(lroundf(100.0f * sensor_range)); //convert to cm

	if (sensor_reading < sensor_range) {
		if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
		     && sensor_range_cm <= _data_maxranges[map_index])
		    || _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]) {

			return true;
		}

	} else {
		if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]
		     && sensor_range_cm >= _data_maxranges[map_index])
		    || (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
			&& sensor_range_cm == _data_maxranges[map_index])) {

			return true;
		}
	}

	return false;
}

void
CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor, const Quatf &vehicle_attitude)
{
	// clamp at maximum sensor range
	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	// negative values indicate out of range but valid measurements.
	if (fabsf(distance_sensor.current_distance - -1.f) < FLT_EPSILON && distance_sensor.signal_quality == 0) {
		distance_reading = distance_sensor.max_distance;
	}

	// discard values below min range
	if (distance_reading > distance_sensor.min_distance) {
		float sensor_yaw_body_rad = ObstacleMath::sensor_orientation_to_yaw_offset(static_cast<ObstacleMath::SensorOrientation>
					    (distance_sensor.orientation), distance_sensor.q);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)round((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) / BIN_SIZE);
		int upper_bound = (int)round((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) / BIN_SIZE);

		if (distance_reading < distance_sensor.max_distance) {
			ObstacleMath::project_distance_on_horizontal_plane(distance_reading, sensor_yaw_body_rad, vehicle_attitude);
		}

		uint16_t sensor_range = static_cast<uint16_t>(lroundf(100.0f * distance_sensor.max_distance)); // convert to cm

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = ObstacleMath::wrap_bin(bin, BIN_COUNT);

			if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading)) {
				_obstacle_map_body_frame.distances[wrapped_bin] = static_cast<uint16_t>(lroundf(100.0f * distance_reading));
				_data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
				_data_maxranges[wrapped_bin] = sensor_range;
			}
		}
	}
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f; // base mode VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.timestamp = getTime();
	_vehicle_command_pub.publish(command);
}
