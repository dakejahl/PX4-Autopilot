# PPK Implementation Plan

## Overview

This document outlines the implementation plan for adding PPK (Post-Processed Kinematics) support to the ARK X20 and ARK RTK GPS (F9P) modules. The implementation involves streaming raw RTCM data from the GPS CANnode to the Flight Controller via a new `PPKStream` DroneCAN message.

## Design Decisions (Based on User Input)

- **DroneCAN Message**: Create a new dedicated `PPKStream` DroneCAN message
- **Message Rate**: Start at 1Hz (pending further research)
- **Logging Format**: Log raw RTCM data (pending further research)
- **Enable/Disable**: Use a parameter to enable/configure PPK output
- **Supported Modules**: ARK X20 (ZED-F9P) and ARK RTK GPS (F9P) only

---

## Implementation Parts

### Part 1: Define the PPKStream DroneCAN Message

**Goal**: Create a new DSDL message definition for PPK RTCM streaming.

**Tasks**:
1. Create the DSDL file `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/gnss/106x.PPKStream.uavcan`
   - Allocate an appropriate data type ID (likely 1063, after RTCMStream at 1062)
   - Define message structure with protocol_id and data payload (300 bytes to match MovingBaselineData)
2. Regenerate DroneCAN headers/code if required by the build system

**Deliverable**: New PPKStream DSDL message definition

---

### Part 2: CANnode - GPS Driver RTCM Output Configuration

**Goal**: Configure the F9P to output the required RTCM messages for PPK.

**Tasks**:
1. Add a new parameter to enable PPK RTCM output (e.g., `GPS_PPK_OUTPUT` or similar)
2. Modify `src/drivers/gps/devices/src/ubx.cpp` to:
   - Add configuration for PPK RTCM message output when parameter is enabled
   - Configure RTCM message rates: 1005, 1077, 1087, 1097, 1127, 1230
3. Modify `src/drivers/gps/gps.cpp` to:
   - Handle the new parameter
   - Route RTCM output data to a uORB topic for the DroneCAN publisher

**Deliverable**: F9P configured to output RTCM messages when PPK is enabled

---

### Part 3: CANnode - Create PPK uORB Message and Publisher Infrastructure

**Goal**: Create the uORB topic and data flow for PPK RTCM data on the CANnode.

**Tasks**:
1. Create a new uORB message `msg/PpkRtcmData.msg` (or determine if `gps_inject_data` can be reused with a flag)
2. Modify GPS driver to publish RTCM output to this topic
3. Ensure appropriate queue depth for the uORB topic

**Deliverable**: uORB topic carrying PPK RTCM data on CANnode

---

### Part 4: CANnode - DroneCAN PPKStream Publisher

**Goal**: Create a DroneCAN publisher that broadcasts PPK RTCM data.

**Tasks**:
1. Create `src/drivers/uavcannode/Publishers/PPKStream.hpp`
   - Subscribe to the PPK uORB topic
   - Publish `PPKStream` DroneCAN messages
   - Handle message fragmentation if RTCM messages exceed DroneCAN payload
2. Modify `src/drivers/uavcannode/UavcanNode.cpp` to register the new publisher
3. Update board configs (`boards/ark/f9p-gps/`, `boards/ark/can-gps/`) to enable the PPK publisher

**Deliverable**: PPK RTCM data broadcasting over DroneCAN

---

### Part 5: Flight Controller - DroneCAN PPKStream Subscriber

**Goal**: Receive PPK RTCM data on the Flight Controller via DroneCAN.

**Tasks**:
1. Create `src/drivers/uavcan/sensors/ppk_stream.cpp` (and .hpp)
   - Subscribe to `PPKStream` DroneCAN messages
   - Reassemble fragmented RTCM messages if needed
   - Publish to a uORB topic for logging
2. Modify `src/drivers/uavcan/uavcan_main.cpp` to register the new sensor bridge
3. Create or reuse a uORB message for the received PPK data

**Deliverable**: PPK RTCM data received and available as uORB topic on FC

---

### Part 6: Flight Controller - Logging Integration

**Goal**: Ensure PPK RTCM data is properly logged for post-processing.

**Tasks**:
1. Add the PPK uORB topic to the logger configuration
2. Verify logging format is suitable for PPK post-processing tools
3. Test that log files contain valid RTCM data that can be extracted

**Deliverable**: PPK RTCM data logged in ULog files

---

### Part 7: Testing and Validation

**Goal**: Verify end-to-end PPK data flow and usability.

**Tasks**:
1. Test on ARK X20 (ZED-F9P)
2. Test on ARK RTK GPS (F9P)
3. Verify DroneCAN message transmission and reception
4. Verify logging
5. Extract RTCM data from logs and validate with PPK processing tools (if available)
6. Measure CAN bus utilization and verify no adverse effects on other messages

**Deliverable**: Validated PPK implementation

---

## Implementation Order

1. **Part 1** - Define the DroneCAN message (foundation for everything else)
2. **Part 2** - GPS driver RTCM output (generates the data)
3. **Part 3** - CANnode uORB infrastructure (internal data flow)
4. **Part 4** - CANnode DroneCAN publisher (broadcasts data)
5. **Part 5** - FC DroneCAN subscriber (receives data)
6. **Part 6** - FC logging (stores data)
7. **Part 7** - Testing (validates everything)

Parts 2-3 can potentially be developed in parallel with Part 5-6, as they are on different firmware targets.

---

## Open Research Items

See `claudes_research2.md` for remaining research tasks that should be completed before or during implementation.

---

## Files to Create/Modify Summary

### New Files:
- `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/gnss/1063.PPKStream.uavcan` (DSDL)
- `msg/PpkRtcmData.msg` (uORB, possibly)
- `src/drivers/uavcannode/Publishers/PPKStream.hpp` (CANnode publisher)
- `src/drivers/uavcan/sensors/ppk_stream.cpp` (FC subscriber)
- `src/drivers/uavcan/sensors/ppk_stream.hpp` (FC subscriber header)

### Modified Files:
- `src/drivers/gps/devices/src/ubx.cpp` (F9P RTCM output config)
- `src/drivers/gps/gps.cpp` (PPK param handling, uORB publishing)
- `src/drivers/uavcannode/UavcanNode.cpp` (register publisher)
- `src/drivers/uavcan/uavcan_main.cpp` (register subscriber)
- `boards/ark/f9p-gps/default.px4board` (enable PPK)
- `boards/ark/can-gps/default.px4board` (enable PPK)
- Logger configuration (if needed)
