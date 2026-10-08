# Free space around the vehicle in the obstacle map, swept by the vehicle's body
#
# Each sweep moves a vertical cylinder the size of the vehicle from its position along a
# direction, and measures how far it travels before it overlaps an occupied voxel and how far
# through voxels observed free. The ground the vehicle flies over and a ceiling it flies under
# are left out of the cylinder where it starts, so only moving towards them makes contact.

uint64 timestamp                        # [us] Time since system start
uint64 timestamp_sample                 # [us] Time of the vehicle position the sweeps start from

float32 body_radius                     # [m] Radius of the swept cylinder
float32 max_distance                    # [m] Furthest each sweep travels

uint8 SWEEP_UP = 0
uint8 SWEEP_DOWN = 1
uint8 SWEEP_SETPOINT = 2                # Along the velocity setpoint
uint8 SWEEP_VELOCITY = 3                # Along the estimated velocity
uint8 SWEEP_COUNT = 4

float32[4] direction_north              # Unit direction of each sweep, local NED
float32[4] direction_east
float32[4] direction_down
float32[4] contact                      # [m] Travel before the body overlaps an occupied voxel, INFINITY if not within max_distance, NAN if not swept
float32[4] observed                     # [m] Travel through voxels all observed free, at most max_distance, NAN if not swept
uint8[4] contact_face                   # [@enum FACE] Side of the body the contact is on

uint8 FACE_SIDE = 0
uint8 FACE_TOP = 1
uint8 FACE_BOTTOM = 2

# TOPICS obstacle_clearance
