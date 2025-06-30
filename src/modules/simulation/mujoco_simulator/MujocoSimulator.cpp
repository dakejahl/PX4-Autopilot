#include "MujocoSimulator.hpp"

#include <uORB/Subscription.hpp>

#include <px4_platform_common/getopt.h>

MujocoSimulator::MujocoSimulator(const char *mujoco_model_path) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	char error[1000] = "Could not load binary model";
	_model = mj_loadXML(mujoco_model_path, 0, error, 1000);

	if (!_model) {
		PX4_INFO("Error: %s", error);
		PX4_INFO("MuJoCo: Could not load model '%s'", mujoco_model_path);
		px4_usleep(25000);
		return;
	}

	_data = mj_makeData(_model);

	if (!_data) {
		PX4_ERR("MuJoCo: Could not create mjData structure.");
		mj_deleteModel(_model);
		_model = nullptr;
		return;
	}

	_model->opt.timestep = static_cast<mjtNum>(_sim_step_interval_us) / 1e6;

	PX4_INFO("MuJoCo model loaded and data structure created successfully.");
	_initialized = true;
}

MujocoSimulator::~MujocoSimulator()
{
	if (_data) {
		mj_deleteData(_data);
	}

	if (_model) {
		mj_deleteModel(_model);
	}
}

bool MujocoSimulator::init()
{
	if (!_initialized) {
		PX4_ERR("MuJoCo initialization failed in constructor.");
		return false;
	}

	// Schedule the first run.
	ScheduleOnInterval(_sim_step_interval_us);
	return OK;
}


int MujocoSimulator::task_spawn(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage("missing model file path");
		return -1;
	}

	MujocoSimulator *instance = new MujocoSimulator(argv[1]);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (instance->_initialized) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() != PX4_OK) {
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void MujocoSimulator::Run()
{
	if (!_initialized) {
		return;
	}

	if (should_exit()) {
		ScheduleClear();

		//_mixing_interface_esc.stop();
		//_mixing_interface_servo.stop();
		//_mixing_interface_wheel.stop();
		//_gimbal.stop();

		exit_and_cleanup();
		return;
	}

	// Update actuator inputs from PX4
	update_actuators();

	// Step the simulation
	mj_step(_model, _data);

	// Publish time
	publish_time();

	// Publish sensor data to PX4
	publish_sensors();

	// Sleep to maintain real-time simulation speed
	//px4_usleep(4000); // Corresponds to a 250Hz update rate

	//PX4_INFO("MuJoCo simulation stopped");
}

void MujocoSimulator::update_actuators()
{
	actuator_outputs_s act_out;

	if (_actuator_outputs_sub.update(&act_out)) {
		// Apply actuator forces to the MuJoCo simulation
		// This is highly dependent on your MJCF model definition.
		// The example assumes the actuators are the first N actuators in the model.
		for (int i = 0; i < _model->nu && i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; ++i) {
			_data->ctrl[i] = act_out.output[i];
		}
	}
}

void MujocoSimulator::publish_time()
{
	struct timespec ts;
	double sec;
	double nsec = std::modf(_data->time, &sec);
	
	// Convert integer part to seconds
	ts.tv_sec = static_cast<time_t>(sec);
	
	// Convert fractional part to nanoseconds
	// Ensure nanoseconds are positive, even if total_time_seconds is negative
	ts.tv_nsec = static_cast<long>(std::abs(nsec) * 1e9);
  
	if (!_realtime_clock_set) {
		// Set initial real time clock at startup
		px4_clock_settime(CLOCK_REALTIME, &ts);
		PX4_INFO_RAW("MujocoSIM: inital real time clock ime set.\n");
		_realtime_clock_set = true;

	} else {
		// Keep monotonic clock synchronized
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}
}

void MujocoSimulator::publish_sensors()
{
	const hrt_abstime now = hrt_absolute_time();

	// --- IMU (Accelerometer & Gyro) ---
	// This assumes your MJCF model has sensors named "accelerometer" and "gyro"
	int accel_id = mj_name2id(_model, mjOBJ_SENSOR, "accelerometer");
	int gyro_id = mj_name2id(_model, mjOBJ_SENSOR, "gyro");

	if (accel_id >= 0) {
		sensor_accel_s accel_msg{};
		accel_msg.timestamp = now;
		accel_msg.x = _data->sensordata[_model->name_sensoradr[accel_id]];
		accel_msg.y = _data->sensordata[_model->name_sensoradr[accel_id] + 1];
		accel_msg.z = _data->sensordata[_model->name_sensoradr[accel_id] + 2];
		_sensor_accel_pub.publish(accel_msg);
	}

	if (gyro_id >= 0) {
		sensor_gyro_s gyro_msg{};
		gyro_msg.timestamp = now;
		gyro_msg.x = _data->sensordata[_model->name_sensoradr[gyro_id]];
		gyro_msg.y = _data->sensordata[_model->name_sensoradr[gyro_id] + 1];
		gyro_msg.z = _data->sensordata[_model->name_sensoradr[gyro_id] + 2];
		_sensor_gyro_pub.publish(gyro_msg);
	}

	// --- Magnetometer ---
	int mag_id = mj_name2id(_model, mjOBJ_SENSOR, "magnetometer");
	if (mag_id >= 0) {
		sensor_mag_s mag_msg{};
		mag_msg.timestamp = now;
		mag_msg.x = _data->sensordata[_model->name_sensoradr[mag_id]];
		mag_msg.y = _data->sensordata[_model->name_sensoradr[mag_id] + 1];
		mag_msg.z = _data->sensordata[_model->name_sensoradr[mag_id] + 2];
		_sensor_mag_pub.publish(mag_msg);
	}

	// --- Barometer ---
	int baro_id = mj_name2id(_model, mjOBJ_SENSOR, "barometer");
	if (baro_id >= 0) {
		sensor_baro_s baro_msg{};
		baro_msg.timestamp = now;
		baro_msg.pressure = _data->sensordata[_model->name_sensoradr[baro_id]];
		// MuJoCo doesn't typically simulate temperature, so a standard value is used.
		baro_msg.temperature = 25.0f; // Standard temperature
		_sensor_baro_pub.publish(baro_msg);
	}

    // --- Ground Truth Attitude and Position (for EKF) ---
    // This is crucial for the EKF to converge in a simulation environment.
    // We publish ground truth from the simulator directly.

    // Attitude
    int body_id = mj_name2id(_model, mjOBJ_BODY, "drone_body"); // Assuming your main body is named "drone_body"
    if (body_id >= 0) {
        vehicle_attitude_s att_msg{};
        att_msg.timestamp = now;
        mjtNum* quat = _data->xquat + (body_id * 4);
        att_msg.q[0] = quat[0]; // w
        att_msg.q[1] = quat[1]; // x
        att_msg.q[2] = quat[2]; // y
        att_msg.q[3] = quat[3]; // z
        _attitude_pub.publish(att_msg);

        // Local Position
        vehicle_local_position_s lpos_msg{};
        lpos_msg.timestamp = now;
        mjtNum* pos = _data->xpos + (body_id * 3);
        lpos_msg.x = pos[0];
        lpos_msg.y = pos[1];
        lpos_msg.z = pos[2];
        _local_pos_pub.publish(lpos_msg);

        // Global Position (requires a reference point)
        // For simplicity, we'll use the local position directly.
        // A more advanced simulation would handle a global frame.
        vehicle_global_position_s gpos_msg{};
        gpos_msg.timestamp = now;
        gpos_msg.lat = 47.397742; // Example: Zurich
        gpos_msg.lon = 8.545594;
        gpos_msg.alt = pos[2] + 488; // Altitude above sea level
        _global_pos_pub.publish(gpos_msg);
    }
}

int MujocoSimulator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MujocoSimulator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
			R"DESC_STR(
### Description
PX4-MuJoCo bridge for Software-in-the-Loop (SITL) simulation.
This module runs the MuJoCo physics engine internally, loads a model,
and bridges sensor/actuator data with the PX4 flight stack via uORB.

It replaces the need for an external simulator like Gazebo by integrating
the dynamics calculations directly into a PX4 module.

### Usage
Start the bridge by providing the path to a MuJoCo XML model file.
$ mujoco_simulator start <path_to_model.xml>
)DESC_STR");

	PRINT_MODULE_USAGE_NAME("mujoco_simulator", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("<file>", "MuJoCo XML model file", false);
	return 0;
}

extern "C" __EXPORT int mujoco_simulator_main(int argc, char *argv[])
{
	return MujocoSimulator::main(argc, argv);
}
