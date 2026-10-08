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

#pragma once

#include "ObstacleGrid.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/obstacle_clearance.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/obstacle_map_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/range_image.h>
#include <uORB/topics/range_image_info.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class ObstacleMap : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ObstacleMap();
	~ObstacleMap() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void updateParameters();
	void updatePose();
	void insertTiles();
	bool mapIsFresh(hrt_abstime now) const;
	void publishObstacleDistance(hrt_abstime now);
	void publishClearance(hrt_abstime now);
	void publishStatus(hrt_abstime now);

	struct PositionSample {
		hrt_abstime timestamp;
		matrix::Vector3f position;
	};

	struct AttitudeSample {
		hrt_abstime timestamp;
		matrix::Quatf attitude;
	};

	bool positionAt(hrt_abstime timestamp, matrix::Vector3f &position) const;
	bool attitudeAt(hrt_abstime timestamp, matrix::Quatf &attitude) const;

	static constexpr int kHistory = 32; // 0.64 s of poses at the 50 Hz schedule
	static constexpr hrt_abstime kSchedule = 20_ms;
	static constexpr hrt_abstime kObstacleDistanceInterval = 50_ms;
	static constexpr hrt_abstime kStatusInterval = 1_s;
	static constexpr hrt_abstime kTileTimeout = 500_ms;
	static constexpr float kMinSweepSpeed = 0.1f; // [m/s] slower, the direction of motion is noise
	static constexpr float kVerticalSweepMargin = 2.f; // [voxels] added to the radius swept up and down
	static constexpr int kBins = sizeof(obstacle_distance_s::distances) / sizeof(obstacle_distance_s::distances[0]);

	obstacle_map::ObstacleGrid _grid;

	PositionSample _positions[kHistory] {};
	AttitudeSample _attitudes[kHistory] {};
	int _position_count{0};
	int _position_newest{0};
	int _attitude_count{0};
	int _attitude_newest{0};

	vehicle_local_position_s _local_position{};
	bool _local_position_valid{false};
	matrix::Quatf _attitude{};
	bool _attitude_valid{false};
	bool _reset_counters_known{false};

	range_image_info_s _info{};
	bool _info_valid{false};

	hrt_abstime _last_tile{0};
	hrt_abstime _last_obstacle_distance{0};
	hrt_abstime _last_status{0};
	float _vertical_margin{0.f};
	uint32_t _rays_inserted{0};
	uint32_t _tiles_dropped{0};

	uORB::SubscriptionCallbackWorkItem _range_image_sub{this, ORB_ID(range_image)};
	uORB::Subscription _range_image_info_sub{ORB_ID(range_image_info)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<obstacle_clearance_s> _clearance_pub{ORB_ID(obstacle_clearance)};
	uORB::Publication<obstacle_map_status_s> _status_pub{ORB_ID(obstacle_map_status)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _insert_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": insert tile")};
	perf_counter_t _sectors_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sectors")};
	perf_counter_t _sweeps_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sweeps")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::OMAP_VOX_SIZE>) _param_omap_vox_size,
		(ParamFloat<px4::params::OMAP_VEH_RAD>) _param_omap_veh_rad,
		(ParamFloat<px4::params::OMAP_VEH_HGT>) _param_omap_veh_hgt,
		(ParamInt<px4::params::OMAP_HIT>) _param_omap_hit,
		(ParamInt<px4::params::OMAP_MISS>) _param_omap_miss,
		(ParamInt<px4::params::OMAP_OCC_THR>) _param_omap_occ_thr,
		(ParamInt<px4::params::OMAP_FREE_THR>) _param_omap_free_thr,
		(ParamInt<px4::params::OMAP_LO_MIN>) _param_omap_lo_min
	)
};
