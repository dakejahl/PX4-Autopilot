uint64 timestamp            # time since system start (microseconds)

float32 temperature_cells       # [C]      : Pack mounted thermistor (preferably installed between cells), NAN: field not provided
float32 temperature_pcb         # [C]      : Battery PCB temperature (likely output FET(s) or current sense resistor), NAN: field not provided
float32 temperature_other       # [C]      : Application dependent, NAN: field not provided
float32 current                 # [Ampere] : Positive: defined as a discharge current. Negitive: defined as a charging current, NAN: field not provided
float32 current_filtered        #
float32 voltage                 # [Volt]
uint32 capacity_remaining       # [mAh]
uint8 state_of_charge           # [%]      : 0-100%

float32[12] cell_voltages       # [Volt]

uint8[50] name                  # Formatted as manufacturer_product
uint8[16] serial_number         # Serial number in ASCII characters, 0 terminated
uint8[11] manufacture_date      # Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated
uint32 design_capacity          # [mAh] Fully Charged Design Capacity. UINT32_MAX: field not provided.
uint32 actual_capacity          # [mAh] Predicted battery capacity when fully charged (accounting for battery degradation)
uint16 cycle_count              # UINT16_MAX: field not provided.
uint8 state_of_health           # % state of health (0 - 100) UINT8_MAX: field not provided
uint8 cells_in_series
uint32 status_flags

uint32 STATUS_FLAG_READY_TO_USE                             = 1
uint32 STATUS_FLAG_CHARGING                                 = 2
uint32 STATUS_FLAG_CELL_BALANCING                           = 4
uint32 STATUS_FLAG_FAULT_CELL_IMBALANCE                     = 8
uint32 STATUS_FLAG_AUTO_DISCHARGING                         = 16
uint32 STATUS_FLAG_REQUIRES_SERVICE                         = 32
uint32 STATUS_FLAG_BAD_BATTERY                              = 64
uint32 STATUS_FLAG_PROTECTIONS_ENABLED                      = 128
uint32 STATUS_FLAG_FAULT_PROTECTION_SYSTEM                  = 256
uint32 STATUS_FLAG_FAULT_OVER_VOLT                          = 512
uint32 STATUS_FLAG_FAULT_UNDER_VOLT                         = 1024
uint32 STATUS_FLAG_FAULT_OVER_TEMP                          = 2048
uint32 STATUS_FLAG_FAULT_UNDER_TEMP                         = 4096
uint32 STATUS_FLAG_FAULT_OVER_CURRENT                       = 8192
uint32 STATUS_FLAG_FAULT_SHORT_CIRCUIT                      = 16384
uint32 STATUS_FLAG_FAULT_INCOMPATIBLE_VOLTAGE               = 32768
uint32 STATUS_FLAG_FAULT_INCOMPATIBLE_FIRMWARE              = 65536
uint32 STATUS_FLAG_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION   = 131072
uint32 STATUS_FLAG_CAPACITY_RELATIVE_TO_FULL                = 262144

