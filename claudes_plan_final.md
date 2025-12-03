# PPK Implementation - Final Plan

## Project Overview

Implement Post-Processed Kinematics (PPK) support for ARK X20 and ARK RTK GPS modules. This involves streaming raw RTCM data from the GPS CANnode to the Flight Controller via DroneCAN for logging and post-processing.

**Data Flow**: GPS Module (F9P) → CANnode GPS Driver → uORB → DroneCAN Publisher → CAN Bus → FC DroneCAN Subscriber → uORB → Logger

---

## Key Design Decisions (From Research)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| DroneCAN Message | New `PPKStream` (ID 1064) | Clean separation, sequential with GNSS messages |
| Payload Size | 300 bytes | Matches MovingBaselineData, handles MSM7 messages |
| Enable Parameter | `GPS_UBX_MODE=7` | Follows existing RTCM mode pattern |
| Update Rate | 5Hz default | Standard for drone PPK/photogrammetry |
| Data Format | Raw RTCM3 | Tools handle RTCM3→RINEX conversion natively |
| Logging | New `PpkRtcmData.msg` topic | Similar to `GpsDump.msg` pattern |

---

## Implementation Tasks

### Phase 1: Foundation (DroneCAN Message)

#### Task 1.1: Create PPKStream DSDL Definition
**Files**: `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/gnss/1064.PPKStream.uavcan`

**Work**:
- Create new DSDL file with data type ID 1064
- Define message structure:
  ```
  uint8 PROTOCOL_ID_UNKNOWN = 0
  uint8 PROTOCOL_ID_RTCM3   = 3
  uint8 protocol_id
  uint8[<=300] data
  ```

**Acceptance Criteria**:
- DSDL file exists and follows DroneCAN conventions
- Build system can generate headers from the DSDL

---

### Phase 2: CANnode GPS Driver (RTCM Output)

#### Task 2.1: Add GPS_UBX_MODE=7 Parameter Support
**Files**: `src/drivers/gps/gps.cpp`, `src/drivers/gps/module.yaml`

**Work**:
- Add PPK mode (value 7) to GPS_UBX_MODE parameter
- Handle mode 7 in GPS driver initialization

**Acceptance Criteria**:
- GPS_UBX_MODE=7 is recognized by the driver
- Parameter documentation is updated

---

#### Task 2.2: Configure F9P for RTCM Output in PPK Mode
**Files**: `src/drivers/gps/devices/src/ubx.cpp`

**Work**:
- When PPK mode is enabled, configure F9P to output RTCM messages:
  - 1005 (Station coordinates) @ 5Hz
  - 1077 (GPS MSM7) @ 5Hz
  - 1087 (GLONASS MSM7) @ 5Hz
  - 1097 (Galileo MSM7) @ 5Hz
  - 1127 (BeiDou MSM7) @ 5Hz
  - 1230 (GLONASS bias) @ 5Hz
- Use UBX-CFG-VALSET to configure message rates
- Enable RTCM3 output protocol on appropriate port

**Acceptance Criteria**:
- F9P outputs RTCM messages when PPK mode enabled
- Messages output at configured rate

---

#### Task 2.3: Create uORB Topic for PPK Data
**Files**: `msg/PpkRtcmData.msg`, `msg/CMakeLists.txt`

**Work**:
- Create new uORB message:
  ```
  uint64 timestamp
  uint8 len
  uint8[300] data
  uint8 ORB_QUEUE_LENGTH = 16
  ```
- Add to CMakeLists.txt

**Acceptance Criteria**:
- uORB message compiles
- Queue depth sufficient for 5Hz burst traffic

---

#### Task 2.4: GPS Driver Publishes RTCM to uORB
**Files**: `src/drivers/gps/gps.cpp`, `src/drivers/gps/devices/src/ubx.cpp`

**Work**:
- Parse incoming RTCM stream from F9P
- Buffer complete RTCM messages
- Publish to `ppk_rtcm_data` uORB topic

**Acceptance Criteria**:
- Complete RTCM messages published to uORB
- No message corruption or truncation

---

### Phase 3: CANnode DroneCAN Publisher

#### Task 3.1: Create PPKStream Publisher
**Files**: `src/drivers/uavcannode/Publishers/PPKStream.hpp`

**Work**:
- Create new publisher class (follow `MovingBaselineData.hpp` pattern)
- Subscribe to `ppk_rtcm_data` uORB topic
- Publish `uavcan::equipment::gnss::PPKStream` messages
- Handle message fragmentation if RTCM > 300 bytes

**Acceptance Criteria**:
- Publisher broadcasts PPK data over DroneCAN
- Large messages properly fragmented

---

#### Task 3.2: Register PPKStream Publisher
**Files**: `src/drivers/uavcannode/UavcanNode.cpp`

**Work**:
- Include PPKStream.hpp
- Add PPKStream publisher to the publisher list
- Conditional compilation based on board config

**Acceptance Criteria**:
- Publisher instantiated on CANnode startup

---

#### Task 3.3: Enable PPK in ARK Board Configs
**Files**:
- `boards/ark/can-rtk-gps/default.px4board`

**Work**:
- Add build flag to enable PPK publisher
- Ensure board has sufficient resources

**Acceptance Criteria**:
- ARK CANnode builds with PPK support

---

### Phase 4: Flight Controller DroneCAN Subscriber

#### Task 4.1: Create PPKStream Subscriber/Bridge
**Files**:
- `src/drivers/uavcan/sensors/ppk_stream.hpp`
- `src/drivers/uavcan/sensors/ppk_stream.cpp`

**Work**:
- Create sensor bridge class (follow existing patterns in `sensors/`)
- Subscribe to `uavcan::equipment::gnss::PPKStream`
- Reassemble fragmented messages if needed
- Publish to `ppk_rtcm_data` uORB topic

**Acceptance Criteria**:
- FC receives PPK DroneCAN messages
- Data available as uORB topic

---

#### Task 4.2: Register PPKStream Subscriber
**Files**: `src/drivers/uavcan/uavcan_main.cpp`

**Work**:
- Include ppk_stream.hpp
- Register the sensor bridge in start sequence
- Add to uavcan sensor list

**Acceptance Criteria**:
- Subscriber active when uavcan driver starts

---

### Phase 5: Logging Integration

#### Task 5.1: Add PPK Topic to Logger
**Files**: `src/modules/logger/logged_topics.cpp` (or equivalent config)

**Work**:
- Add `ppk_rtcm_data` as optional logged topic
- Ensure appropriate logging rate

**Acceptance Criteria**:
- PPK data logged to ULog file
- Timestamps preserved

---

#### Task 5.2: Create Log Extraction Tool/Documentation
**Files**: Documentation or utility script

**Work**:
- Document how to extract raw RTCM data from ULog
- Create extraction script if needed (pyulog-based)
- Document PPK post-processing workflow

**Acceptance Criteria**:
- User can extract RTCM data from flight log
- RTCM data usable with RTKLIB/Emlid Studio

---

### Phase 6: Testing and Validation

#### Task 6.1: Unit Testing
**Work**:
- Test RTCM message parsing
- Test DroneCAN message encoding/decoding
- Test fragmentation/reassembly

---

#### Task 6.2: Hardware Integration Testing
**Work**:
- Test on ARK X20 (ZED-F9P)
- Test on ARK RTK GPS
- Verify DroneCAN traffic with analyzer
- Measure CAN bus utilization

---

#### Task 6.3: End-to-End PPK Validation
**Work**:
- Capture flight log with PPK data
- Extract RTCM data
- Process with PPK software
- Verify position accuracy improvement

---

## Task Dependencies

```
1.1 (DSDL) ─────────────────────────────────────────────────┐
                                                             │
2.1 (Param) ──► 2.2 (F9P Config) ──► 2.4 (uORB Publish) ───┤
                                           │                 │
2.3 (uORB Msg) ────────────────────────────┘                │
                                                             ▼
                                               3.1 (Publisher) ──► 3.2 (Register) ──► 3.3 (Board)
                                                             │
                                                             ▼
                                               4.1 (Subscriber) ──► 4.2 (Register)
                                                             │
                                                             ▼
                                               5.1 (Logger) ──► 5.2 (Extraction)
                                                             │
                                                             ▼
                                               6.1 ──► 6.2 ──► 6.3 (Testing)
```

---

## Estimated RTCM Data Volume

| Message | Size (bytes) | Rate | Bandwidth |
|---------|-------------|------|-----------|
| 1005 | ~20 | 5Hz | 100 B/s |
| 1077 | ~200-450 | 5Hz | 1000-2250 B/s |
| 1087 | ~200-400 | 5Hz | 1000-2000 B/s |
| 1097 | ~200-400 | 5Hz | 1000-2000 B/s |
| 1127 | ~200-400 | 5Hz | 1000-2000 B/s |
| 1230 | ~50 | 5Hz | 250 B/s |
| **Total** | | | **~5-10 KB/s** |

This is within CAN bus capacity (~1 Mbit/s = 125 KB/s theoretical, ~50-80 KB/s practical with overhead).

---

## Reference Files

### Existing Patterns to Follow:
- `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` - Publisher pattern
- `src/drivers/uavcannode/Subscribers/RTCMStream.hpp` - Subscriber pattern
- `src/drivers/uavcan/sensors/gnss.cpp` - FC-side GNSS handling
- `/home/jake/code/ark/private_px4/src/drivers/uavcannode/Publishers/SeptentrioSbfChunk.hpp` - Similar streaming implementation

### RTCM Messages (in order of priority):
1. 1077 - GPS MSM7 (essential)
2. 1087 - GLONASS MSM7
3. 1097 - Galileo MSM7
4. 1127 - BeiDou MSM7
5. 1005 - Station coordinates
6. 1230 - GLONASS bias

---

## Implementation Order

**Recommended order for incremental development and testing:**

1. Task 1.1 - DSDL (foundation for all DroneCAN work)
2. Task 2.3 - uORB message (foundation for data flow)
3. Task 2.1, 2.2 - GPS parameter and F9P configuration
4. Task 2.4 - GPS driver RTCM publishing
5. Task 3.1, 3.2, 3.3 - CANnode publisher (can test DroneCAN output)
6. Task 4.1, 4.2 - FC subscriber (end-to-end data flow complete)
7. Task 5.1, 5.2 - Logging integration
8. Tasks 6.x - Testing and validation

---

## Notes

- All code should follow PX4 coding style
- Add appropriate license headers to new files
- Keep changes minimal and focused on PPK functionality
