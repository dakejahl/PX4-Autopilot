# Summary of the persistent obstacle map and a horizontal slice through it at the vehicle's height

uint64 timestamp                        # [us] Time since system start

float32 voxel_size                      # [m] Edge length of one voxel
uint16 grid_size_xy                     # Voxels along north and along east
uint16 grid_size_z                      # Voxels along down
int32[3] center_voxel                   # Voxel index (north, east, down) of the centre of the map window, in the local frame

uint32 occupied_count                   # Occupied voxels in the window
uint32 free_count                       # Free voxels in the window
uint32 rays_inserted                    # Rays inserted since start

uint8 SLICE_SIZE = 64                   # Cells along each side of the slice
uint8 CELL_UNKNOWN = 0
uint8 CELL_FREE = 1
uint8 CELL_OCCUPIED = 2
int32 slice_down_voxel                  # Voxel index along down of the slice
uint8[1024] slice                       # SLICE_SIZE x SLICE_SIZE cells of 2 bits, centred on center_voxel. Cell (n, e) is at bit 2 * (n * SLICE_SIZE + e).

# TOPICS obstacle_map_status
