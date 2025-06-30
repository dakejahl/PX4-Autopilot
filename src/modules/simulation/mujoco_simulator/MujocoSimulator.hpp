#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/drivers/device/Device.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

// MuJoCo headers
#include <mujoco/mujoco.h>

using namespace time_literals;

class MujocoSimulator : public ModuleBase<MujocoSimulator>, public ModuleParams, public px4::ScheduledWorkItem

{
public:
	MujocoSimulator(const char *mujoco_model_path);
	~MujocoSimulator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	bool init();

private:
	// MuJoCo model and data
	mjModel* _model{nullptr};
	mjData*  _data{nullptr};

	// UORB publications
	uORB::Publication<sensor_accel_s> _sensor_accel_pub{ORB_ID(sensor_accel)};
	uORB::Publication<sensor_gyro_s>  _sensor_gyro_pub{ORB_ID(sensor_gyro)};
	uORB::Publication<sensor_mag_s>   _sensor_mag_pub{ORB_ID(sensor_mag)};
	uORB::Publication<sensor_baro_s>  _sensor_baro_pub{ORB_ID(sensor_baro)};

	// Estimated data publications
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s>	_global_pos_pub{ORB_ID(vehicle_global_position)};

	// Subscriptions
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	//uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	// Private methods
	void publish_sensors();
	void publish_time();
	void update_actuators();

	bool _realtime_clock_set{false};

	// --- Simulation Parameters ---
	const hrt_abstime _sim_step_interval_us{4000}; // Corresponds to 250 Hz, adjust as needed
	// Prevent running logic until initialized
	bool _initialized{false};
};
