#
# IMU consistency metrics. Each IMU is compared with the per-axis median of all live IMUs. Zero for an unpopulated slot.
#
uint64 timestamp # time since system start (microseconds)

uint32 accel_device_id_primary       # current primary accel device id for reference

uint32[4] accel_device_ids
float32[4] accel_inconsistency_m_s_s # magnitude of acceleration difference between IMU instance and median in m/s^2.
float32[4] accel_accumulated_error_m_s # accel inconsistency beyond SENS_IMU_ACC, accumulated over time (m/s)
bool[4] accel_healthy
uint8[4] accel_priority
bool accel_fault_detected            # an accel accumulated more than SENS_IMU_VEL
uint32 accel_device_id_faulty        # the faulty accel, 0 if fewer than three accels cannot tell which one

uint32 gyro_device_id_primary       # current primary gyro device id for reference

uint32[4] gyro_device_ids
float32[4] gyro_inconsistency_rad_s # magnitude of angular rate difference between IMU instance and median in (rad/s).
float32[4] gyro_accumulated_error_rad # gyro inconsistency beyond SENS_IMU_RAT, accumulated over time (rad)
bool[4] gyro_healthy
uint8[4] gyro_priority
bool gyro_fault_detected            # a gyro accumulated more than SENS_IMU_ANG
uint32 gyro_device_id_faulty        # the faulty gyro, 0 if fewer than three gyros cannot tell which one
