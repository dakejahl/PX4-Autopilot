uint64 timestamp            # time since system start (microseconds)

float32 temperature             # [C]      : Pack mounted thermistor (preferably installed between cells), NAN: field not provided
float32 current                 # [Ampere] : Postive: defined as a discharge current. Negitive: deifined as a charging current, NAN: field not provided
float32 voltage                 # [Volt]
uint32 capacity_consumed        # [mAh]
uint32 capacity_remaining       # [mAh]

float32[12] cell_voltages       # [Volt]

uint8[50] name                  # Formatted as manufacturer_product
uint8[16] serial_number         # Serial number in ASCII characters, 0 terminated
uint8[11] manufacture_date      # Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated
uint32 design_capacity          # [mAh] Fully Charged Design Capacity. UINT32_MAX: field not provided.
uint32 actual_capacity          # [mAh] Predicted battery capacity when fully charged (accounting for battery degradation)
uint16 cycle_count              # UINT16_MAX: field not provided.
uint8 state_of_health           # % state of health (0 - 100) UINT8_MAX: field not provided

uint32 status_flags

uint32 STATUS_FLAG_IN_USE               = 1
uint32 STATUS_FLAG_READY_TO_USE         = 2
uint32 STATUS_FLAG_CHARGING             = 4
uint32 STATUS_FLAG_OVER_TEMP            = 8
uint32 STATUS_FLAG_UNDER_TEMP           = 16
uint32 STATUS_FLAG_OVER_VOLT            = 32
uint32 STATUS_FLAG_UNDER_VOLT           = 64
uint32 STATUS_FLAG_OVER_CURRENT         = 128
uint32 STATUS_FLAG_SHORT_CIRCUIT        = 256
uint32 STATUS_FLAG_SAFETY_FAULT         = 512
uint32 STATUS_FLAG_CELL_IMBALANCE       = 1024
uint32 STATUS_FLAG_CELL_BALANCING       = 2048
uint32 STATUS_FLAG_CELL_FAULT           = 4096
uint32 STATUS_FLAG_PROTECTIONS_ENABLED  = 8192
uint32 STATUS_FLAG_REQUIRES_SERVICE     = 16384
