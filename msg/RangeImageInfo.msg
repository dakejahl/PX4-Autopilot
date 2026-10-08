# Geometry, range encoding and mounting of one range image stream
#
# The fields mirror the DroneCAN RangeImageInfo message, so a bridge copies them across.
# Sensor frame: x forward (boresight), y right, z down.
#
# PROJECTION_ANGULAR: azimuth = x_start + col * x_step, positive right
#                     elevation = y_start + row * y_step, positive up
#                     direction = (cos(el) cos(az), cos(el) sin(az), -sin(el))
# PROJECTION_PINHOLE: u = x_start + col * x_step
#                     v = y_start + row * y_step
#                     direction = normalize(1, u, -v)
#
# When row_angle_count is non-zero, row_angle[row] replaces y_start + row * y_step.

uint64 timestamp                        # [us] Time since system start

uint32 device_id                        # Unique device ID of the sensor
uint8 config_id                         # Increments whenever any field below changes

uint8 sensor_type                       # [@enum SENSOR_TYPE]
uint8 SENSOR_TYPE_MULTIZONE_TOF = 0
uint8 SENSOR_TYPE_FLASH_LIDAR = 1
uint8 SENSOR_TYPE_DEPTH_CAMERA = 2
uint8 SENSOR_TYPE_SCANNING_2D = 3
uint8 SENSOR_TYPE_SCANNING_3D = 4
uint8 SENSOR_TYPE_RADAR = 5
uint8 SENSOR_TYPE_OTHER = 255

uint16 num_rows
uint16 num_cols

uint8 projection                        # [@enum PROJECTION]
uint8 PROJECTION_ANGULAR = 0            # x and y are azimuth and elevation in degrees
uint8 PROJECTION_PINHOLE = 1            # x and y are tangents on the image plane at unit distance

uint8 range_type                        # [@enum RANGE_TYPE]
uint8 RANGE_TYPE_RADIAL = 0             # Distance along the zone direction
uint8 RANGE_TYPE_DEPTH = 1              # Distance along sensor x

uint8 zone_order                        # [@enum ZONE_ORDER]
uint8 ZONE_ORDER_ROW_MAJOR = 0          # zone = row * num_cols + col
uint8 ZONE_ORDER_COLUMN_MAJOR = 1       # zone = col * num_rows + row

float32 x_start                         # Column 0 azimuth [deg] or u [-]
float32 x_step                          # Column step [deg] or [-]
float32 y_start                         # Row 0 elevation [deg] or v [-]
float32 y_step                          # Row step [deg] or [-]

float32[4] q_body_sensor                # Rotation from the sensor frame to the body frame (FRD), w x y z. All zeros when not known to the sensor.
float32[3] position_body                # [m] Sensor position in the body frame (FRD)

uint8 range_lsb_mm                      # [mm] Value of one count in RangeImage.ranges
float32 range_min                       # [m] Closer targets read RANGE_BELOW_MIN
float32 range_max                       # [m] RANGE_NO_RETURN means nothing within this distance
uint8 range_max_reflectivity            # [%] Lowest target reflectivity range_max holds for, 0 when not stated

uint8 row_angle_count                   # Number of valid entries in row_angle, 0 for evenly spaced rows
float32[128] row_angle                  # Per-row elevation [deg] or v [-] for unevenly spaced rows

# TOPICS range_image_info
