uint64 timestamp # Publication/receive time, local HRT (us)
uint64 timestamp_sample # End of burst interval in local HRT; receive time if mapping invalid
uint64 node_timestamp_us # Original acquisition HRT on source board (us)
uint64 bus_timestamp_us # Acquisition time in shared DroneCAN clock (us), 0 if unknown/local SPI
uint32 device_id # Local flow device ID, or CAN source/interface ID
uint32 gyro_device_id # Source board gyro ID; not the FC CAN gyro ID
uint32 frame_counter # Successful burst reads, including rejected readings; gaps indicate capture/transport loss
uint32 interval_us # Actual burst interval; 0 after reset/SPI error
uint32 shutter # Raw 23-bit shutter, internal oscillator cycles / 4
float32[3] gyro_integral # Integral of uncalibrated sensor_gyro in source board FRD (rad), before SENS_FLOW_ROT; NaN if incomplete
int16 delta_x # Register counts before driver rotation/scaling
int16 delta_y
uint8 motion
uint8 observation # Mode in bits 7:6
uint8 squal_raw
uint8 raw_data_sum
uint8 gyro_samples # Number of interpolated gyro segments used; 0 if incomplete
bool timestamp_sample_valid # True for local SPI or a successfully mapped CAN sample

uint8 ORB_QUEUE_LENGTH = 16
