# Raw RTCM data for Post-Processed Kinematics (PPK).
# Streams RTCM3 observation data from rover GNSS for logging and post-processing.

uint64 timestamp        # time since system start (microseconds)

uint16 len              # length of data in bytes
uint8[300] data         # RTCM message data (max 300 bytes to handle MSM7 messages)

uint8 ORB_QUEUE_LENGTH = 16
