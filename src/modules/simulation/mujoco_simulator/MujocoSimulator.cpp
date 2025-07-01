#include "MujocoSimulator.hpp"

#include <uORB/Subscription.hpp>
#include <px4_platform_common/getopt.h>

// For dlopen, dlsym, dlclose
#include <dlfcn.h>
#include <cmath> // For std::modf

bool MujocoSimulator::load_mujoco_library()
{
	// Attempt to open the MuJoCo shared library.
	// The linker will search for this file in standard library paths.
	// You might need to set LD_LIBRARY_PATH if it's in a custom location.
	_mujoco_handle = dlopen("libmujoco.so", RTLD_LAZY);

	if (!_mujoco_handle) {
		PX4_ERR("Failed to load MuJoCo library: %s", dlerror());
		return false;
	}

	// Clear any existing error
	dlerror();

	// Load all required function pointers from the library
	_mj_loadXML = (decltype(_mj_loadXML))dlsym(_mujoco_handle, "mj_loadXML");
	_mj_makeData = (decltype(_mj_makeData))dlsym(_mujoco_handle, "mj_makeData");
	_mj_deleteModel = (decltype(_mj_deleteModel))dlsym(_mujoco_handle, "mj_deleteModel");
	_mj_deleteData = (decltype(_mj_deleteData))dlsym(_mujoco_handle, "mj_deleteData");
	_mj_step = (decltype(_mj_step))dlsym(_mujoco_handle, "mj_step");
	_mj_name2id = (decltype(_mj_name2id))dlsym(_mujoco_handle, "mj_name2id");

	// Check for errors during symbol loading
	const char *dlsym_error = dlerror();

	if (dlsym_error) {
		PX4_ERR("Failed to load MuJoCo symbol: %s", dlsym_error);
		dlclose(_mujoco_handle);
		_mujoco_handle = nullptr;
		return false;
	}

	// Final check to ensure all pointers are non-null
	if (!_mj_loadXML || !_mj_makeData || !_mj_deleteModel || !_mj_deleteData || !_mj_step || !_mj_name2id) {
		PX4_ERR("One or more required MuJoCo symbols could not be loaded.");
		dlclose(_mujoco_handle);
		_mujoco_handle = nullptr;
		return false;
	}

	PX4_INFO("MuJoCo library loaded and symbols resolved successfully.");
	return true;
}

void MujocoSimulator::close_mujoco_library()
{
	if (_mujoco_handle) {
		dlclose(_mujoco_handle);
		_mujoco_handle = nullptr;
		PX4_INFO("MuJoCo library unloaded.");
	}
}

MujocoSimulator::MujocoSimulator(const char *mujoco_model_path) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// First, try to load the MuJoCo library and its functions
	if (!load_mujoco_library()) {
		PX4_ERR("Aborting initialization: MuJoCo library could not be loaded.");
		return;
	}

	// Now use the function pointers to interact with MuJoCo
	char error[1000] = "Could not load binary model";
	_model = _mj_loadXML(mujoco_model_path, 0, error, 1000);

	if (!_model) {
		PX4_INFO("Error: %s", error);
		PX4_INFO("MuJoCo: Could not load model '%s'", mujoco_model_path);
		px4_usleep(25000);
		return;
	}

	_data = _mj_makeData(_model);

	if (!_data) {
		PX4_ERR("MuJoCo: Could not create mjData structure.");
		_mj_deleteModel(_model);
		_model = nullptr;
		return;
	}

	_model->opt.timestep = static_cast<mjtNum>(_sim_step_interval_us) / 1e6;

	for (int i=0; i< 100; i++)
		_mj_step(_model, _data);
	publish_time();

	PX4_INFO("MuJoCo model loaded and data structure created successfully.");

	updateParams();

	_initialized = true;
}

MujocoSimulator::~MujocoSimulator()
{
	if (_data) {
		_mj_deleteData(_data);
	}

	if (_model) {
		_mj_deleteModel(_model);
	}

	// Unload the library when the simulator is destroyed
	close_mujoco_library();
}

bool MujocoSimulator::init()
{
	if (!_initialized) {
		PX4_ERR("MuJoCo initialization failed in constructor.");
		return false;
	}

	// Schedule the first run.
	ScheduleOnInterval(_sim_step_interval_us);
	//ScheduleNow();

	return true; // Use true instead of OK for bool
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

		if (instance->init()) {
			return PX4_OK;

		} else {
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}

	} else {
		PX4_ERR("MujocoSimulator initialization failed.");
		delete instance;
		return PX4_ERROR;
	}
}

void MujocoSimulator::Run()
{
	PX4_INFO("mujoco run");
	if (!_initialized) {
		return;
	}

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Update actuator inputs from PX4
	update_actuators();

	// Step the simulation using the function pointer
	_mj_step(_model, _data);
	PX4_INFO("mj_step %f", _data->time);

	// Publish time
	publish_time();

	// Publish sensor data to PX4
	publish_sensors();

	//updateParams();
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();

		//_mixing_interface_esc.updateParams();
		//_mixing_interface_servo.updateParams();
		//_mixing_interface_wheel.updateParams();
		//_gimbal.updateParams();
	}

	//ScheduleDelayed(_sim_step_interval_us);
}

void MujocoSimulator::update_actuators()
{
	actuator_outputs_s act_out;

	if (_actuator_outputs_sub.update(&act_out)) {
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

	ts.tv_sec = static_cast<time_t>(sec);
	ts.tv_nsec = static_cast<long>(std::abs(nsec) * 1e9);

	if (!_realtime_clock_set) {
		px4_clock_settime(CLOCK_REALTIME, &ts);
		PX4_INFO_RAW("MujocoSIM: initial real time clock time set.\n");
		_realtime_clock_set = true;

	} else {
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}
}

void MujocoSimulator::publish_sensors()
{
	const hrt_abstime now = hrt_absolute_time();

	// --- IMU (Accelerometer & Gyro) ---
	int accel_id = _mj_name2id(_model, mjOBJ_SENSOR, "accelerometer");
	int gyro_id = _mj_name2id(_model, mjOBJ_SENSOR, "gyro");

	if (accel_id >= 0) {
		printf("sending accel\n");
		sensor_accel_s accel_msg{};
		accel_msg.timestamp = now;
		accel_msg.x = _data->sensordata[_model->name_sensoradr[accel_id]];
		accel_msg.y = _data->sensordata[_model->name_sensoradr[accel_id] + 1];
		accel_msg.z = _data->sensordata[_model->name_sensoradr[accel_id] + 2];
		_sensor_accel_pub.publish(accel_msg);
	}

	if (gyro_id >= 0) {
		printf("sending gyro\n");
		sensor_gyro_s gyro_msg{};
		gyro_msg.timestamp = now;
		gyro_msg.x = _data->sensordata[_model->name_sensoradr[gyro_id]];
		gyro_msg.y = _data->sensordata[_model->name_sensoradr[gyro_id] + 1];
		gyro_msg.z = _data->sensordata[_model->name_sensoradr[gyro_id] + 2];
		_sensor_gyro_pub.publish(gyro_msg);
	}

	// --- Magnetometer ---
	int mag_id = _mj_name2id(_model, mjOBJ_SENSOR, "magnetometer");

	if (mag_id >= 0) {
		printf("sending mag\n");
		sensor_mag_s mag_msg{};
		mag_msg.timestamp = now;
		mag_msg.x = _data->sensordata[_model->name_sensoradr[mag_id]];
		mag_msg.y = _data->sensordata[_model->name_sensoradr[mag_id] + 1];
		mag_msg.z = _data->sensordata[_model->name_sensoradr[mag_id] + 2];
		_sensor_mag_pub.publish(mag_msg);
	}

	// --- Barometer ---
	int baro_id = _mj_name2id(_model, mjOBJ_SENSOR, "barometer");

	if (baro_id >= 0) {
		sensor_baro_s baro_msg{};
		baro_msg.timestamp = now;
		baro_msg.pressure = _data->sensordata[_model->name_sensoradr[baro_id]];
		baro_msg.temperature = 288.15f; // Standard temperature
		_sensor_baro_pub.publish(baro_msg);
	}

	// --- Ground Truth Attitude and Position (for EKF) ---
	int body_id = _mj_name2id(_model, mjOBJ_BODY, "drone_body");

	if (body_id >= 0) {
		printf("sending ground truth\n");
		// Attitude
		vehicle_attitude_s att_msg{};
		att_msg.timestamp = now;
		mjtNum *quat = _data->xquat + (body_id * 4);
		att_msg.q[0] = quat[0]; // w
		att_msg.q[1] = quat[1]; // x
		att_msg.q[2] = quat[2]; // y
		att_msg.q[3] = quat[3]; // z
		_attitude_pub.publish(att_msg);

		// Local Position
		vehicle_local_position_s lpos_msg{};
		lpos_msg.timestamp = now;
		mjtNum *pos = _data->xpos + (body_id * 3);
		lpos_msg.x = pos[0];
		lpos_msg.y = pos[1];
		lpos_msg.z = pos[2];
		_local_pos_pub.publish(lpos_msg);

		// Global Position
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
This module runs the MuJoCo physics engine internally by dynamically loading
the shared library at runtime. It loads a model and bridges sensor/actuator
data with the PX4 flight stack via uORB.

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

