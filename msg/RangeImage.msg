# One tile of a range image frame
#
# A frame is published as consecutive tiles in the zone order of the matching RangeImageInfo.
# ranges[k] is zone first_zone + k, for k < num_ranges.
# Capture time of a zone: timestamp_sample + (col(zone) - col(first_zone)) * column_dt.
#
# The fields mirror the DroneCAN RangeImage message, with the capture time as an absolute timestamp.

uint64 timestamp                        # [us] Time since system start
uint64 timestamp_sample                 # [us] Capture time of zone first_zone

uint32 device_id                        # Matches RangeImageInfo.device_id
uint8 config_id                         # Matches RangeImageInfo.config_id
uint8 frame_seq                         # Increments per frame, shared by all tiles of a frame

float32 column_dt                       # [us] Capture time step from one column to the next, 0 when all zones are captured at once

uint32 first_zone                       # Zone index of ranges[0]
uint16 num_ranges                       # Number of valid entries in ranges

uint16 RANGE_NO_RETURN = 0              # Measured, nothing within RangeImageInfo.range_max
uint16 RANGE_MAX_COUNT = 4093           # Largest count that is a distance
uint16 RANGE_BELOW_MIN = 4094           # Something closer than RangeImageInfo.range_min
uint16 RANGE_INVALID = 4095             # No usable measurement

uint16[200] ranges                      # Closest return per zone, in counts of RangeImageInfo.range_lsb_mm

uint8 ORB_QUEUE_LENGTH = 16             # A full-resolution frame of a 54 x 42 zone sensor is 12 tiles

# TOPICS range_image
