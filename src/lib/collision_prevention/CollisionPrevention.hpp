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
 * @file CollisionPrevention.hpp
 * @author Tanja Baumann <tanja@auterion.com>
 * @author Claudio Chies <claudio@chies.com>
 *
 * Collision Prevention: limits the horizontal acceleration setpoint so the vehicle keeps CP_DIST from
 * the obstacles in obstacle_distance and distance_sensor, and the vertical speed so it keeps
 * CP_DIST_V from what the obstacle map holds above and below. It never steers: a command into an
 * obstacle stops short of it, a command past it slides along it.
 */

#pragma once

#include <float.h>

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/collision_constraints.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/obstacle_clearance.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>

#include "AccelerationLimits.hpp"

using namespace time_literals;

class CollisionPrevention : public ModuleParams
{
public:
	CollisionPrevention(ModuleParams *parent);
	~CollisionPrevention() override = default;

	/**
	 * Returns true if Collision Prevention is running
	 */
	bool is_active();

	/**
	 * Computes collision free setpoints
	 * @param setpoint_accel setpoint purely based on sticks, to be modified
	 * @param setpoint_vel current velocity setpoint as information to be able to stop in time, does not get changed
	 */
	void modifySetpoint(matrix::Vector2f &setpoint_accel, const matrix::Vector2f &setpoint_vel);

	/**
	 * Climb and descent speeds the obstacle map's clearance above, below and along the path
	 * allows, as of the last modifySetpoint().
	 * @return false when it limits neither
	 */
	bool verticalSpeedLimits(float &up, float &down) const;

	static constexpr int BIN_COUNT =
		sizeof(obstacle_distance_s::distances) / sizeof(obstacle_distance_s::distances[0]); // 72
	static constexpr int BIN_SIZE = 360 / BIN_COUNT; // cannot be lower than 5 degrees, should divide 360 evenly

protected:
	/** Aggregates the sensor data into an internal obstacle map in body frame */
	void _updateObstacleMap();

	/** Updates the obstacle data based on stale data and calculates values from the map */
	void _updateObstacleData();

	/** Calculate the constrained setpoint considering the current obstacle distances, acceleration setpoint and velocity setpoint */
	void _calculateConstrainedSetpoint(matrix::Vector2f &setpoint_accel, const matrix::Vector2f &setpoint_vel);

	obstacle_distance_s _obstacle_map_body_frame{};
	uint64_t _data_timestamps[BIN_COUNT] {};
	uint16_t _data_maxranges[BIN_COUNT] {}; /**< in cm */

	void _addDistanceSensorData(distance_sensor_s &distance_sensor, const matrix::Quatf &vehicle_attitude);

	/**
	 * Updates obstacle distance message with measurement from offboard
	 * @param obstacle, obstacle_distance message to be updated
	 */
	void _addObstacleSensorData(const obstacle_distance_s &obstacle, const float vehicle_yaw);

	/**
	 * Determines whether a new sensor measurement is used
	 * @param map_index, index of the bin in the internal map the measurement belongs in
	 * @param sensor_range, max range of the sensor in meters
	 * @param sensor_reading, distance measurement in meters
	 */
	bool _enterData(int map_index, float sensor_range, float sensor_reading);

	/** Vertical speed limits and horizontal path limits from obstacle_clearance */
	void _updateClearance(hrt_abstime now);

	/** The internal map as obstacles per sector in the heading frame */
	void _polarObstacles(collision_prevention::PolarObstacles &obstacles) const;

	//Timing functions. Necessary to mock time in the tests
	virtual hrt_abstime getTime();
	virtual hrt_abstime getElapsedTime(const hrt_abstime *ptr);

private:
	bool _data_stale{true}; 		/**< states if the data is stale */
	bool _was_active{false};		/**< states if the collision prevention interferes with the user input */
	bool _obstacle_data_present{false};	/**< states if obstacle data is present */

	float _min_dist_to_keep{};

	static constexpr float PUSH_TIME = 1.f; ///< [s] an obstacle inside CP_DIST is backed away from at the depth per this

	orb_advert_t _mavlink_log_pub{nullptr};	 	/**< Mavlink log uORB handle */

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	collision_prevention::PolarObstacles _obstacles{};
	collision_prevention::LimitWorkspace _limit_workspace{};

	matrix::Vector2f _velocity_estimate{NAN, NAN};
	float _vertical_velocity_estimate{NAN}; ///< [m/s] down

	float _speed_limit_up{INFINITY};	/**< [m/s] from the clearance above and along the path */
	float _speed_limit_down{INFINITY};
	static constexpr int MAX_PATH_LIMITS = collision_prevention::LimitWorkspace::kMaxPathLimits;
	collision_prevention::PathLimit _path_limits[MAX_PATH_LIMITS] {};	/**< local frame */
	int _path_limit_count{0};
	matrix::Quatf _vehicle_attitude{};
	float _vehicle_yaw{0.f};

	uORB::Publication<collision_constraints_s>	_constraints_pub{ORB_ID(collision_constraints)};		/**< constraints publication */
	uORB::Publication<obstacle_distance_s>		_obstacle_distance_fused_pub{ORB_ID(obstacle_distance_fused)};	/**< obstacle_distance publication */
	uORB::Publication<vehicle_command_s>	_vehicle_command_pub{ORB_ID(vehicle_command)};			/**< vehicle command do publication */

	uORB::SubscriptionData<obstacle_distance_s> _sub_obstacle_distance{ORB_ID(obstacle_distance)}; /**< obstacle distances received form a range sensor */
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::SubscriptionData<obstacle_clearance_s> _sub_obstacle_clearance{ORB_ID(obstacle_clearance)};

	static constexpr uint64_t RANGE_STREAM_TIMEOUT_US{500_ms};
	static constexpr uint64_t TIMEOUT_HOLD_US{5_s};

	hrt_abstime	_last_timeout_warning{0};
	hrt_abstime	_time_activated{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CP_DIST>) _param_cp_dist, 		/**< collision prevention keep minimum distance */
		(ParamFloat<px4::params::CP_DIST_V>) _param_cp_dist_v,		/**< distance kept above and below */
		(ParamFloat<px4::params::CP_DELAY>) _param_cp_delay, 		/**< delay of the range measurement data*/
		(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max, 	/**< vehicle maximum jerk*/
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, 	/**< vehicle maximum horizontal acceleration*/
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc, /**< p gain from velocity controller*/
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max	/**< maximum manual yaw rate*/
	)


	/**
	 * Publishes collision_constraints message
	 * @param original_setpoint, setpoint before collision prevention intervention
	 * @param adapted_setpoint, collision prevention adaped setpoint
	 */
	void _publishConstrainedSetpoint(const matrix::Vector2f &original_setpoint, const matrix::Vector2f &adapted_setpoint);

	/**
	 * Publishes obstacle_distance message with fused data from offboard and from distance sensors
	 * @param obstacle, obstacle_distance message to be publsihed
	 */
	void _publishObstacleDistance(obstacle_distance_s &obstacle);

	/**
	 * Publishes vehicle command.
	 */
	void _publishVehicleCmdDoLoiter();

};
