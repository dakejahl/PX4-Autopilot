uint64 timestamp # Publication time, local HRT (us)
uint64 timestamp_sample # Motion_Burst read start, local HRT (us); interval ends here
uint32 device_id
uint32 frame_counter # Successful burst reads since driver start, including rejected readings
uint32 interval_us # Time between successful reads; 0 after reset or SPI error
uint32 shutter # Raw 23-bit shutter, internal oscillator cycles / 4
int16 delta_x # Register counts, before driver rotation/scaling
int16 delta_y
uint8 motion # Raw Motion register
uint8 observation # Raw Observation register, mode in bits 7:6
uint8 squal_raw
uint8 raw_data_sum

uint8 ORB_QUEUE_LENGTH = 16
