# PPK Implementation Research

## Overview

This document summarizes the research conducted for implementing Post-Processed Kinematics (PPK) support for the ARK X20 and ARK RTK GPS modules. The goal is to stream raw RTCM data from the GPS module (CANnode) to the Flight Controller via DroneCAN for logging and later post-processing.

## Requirements Summary

From the initial specification (claude.md):

1. **Direction of data flow**: ARK X20 GPS → DroneCAN → PX4 Flight Controller → Logger
2. **RTCM Messages Required for PPK**:
   - 1005 (Stationary RTK Reference Station ARP)
   - 1077 (GPS MSM7)
   - 1087 (GLONASS MSM7)
   - 1097 (Galileo MSM7)
   - 1127 (BeiDou MSM7)
   - 1230 (GLONASS code-phase biases)

3. **Key constraint**: This is the opposite direction of RTK. In RTK, PX4 sends RTCM corrections TO the GPS. For PPK, the GPS sends raw RTCM observations FROM the GPS to PX4 for logging.

---

## Existing Architecture Analysis

### 1. Existing RTCMStream Message (RTK Direction: FC → GPS)

**Location**: `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/gnss/1062.RTCMStream.uavcan`

```
uint8 PROTOCOL_ID_UNKNOWN = 0
uint8 PROTOCOL_ID_RTCM2   = 2
uint8 PROTOCOL_ID_RTCM3   = 3
uint8 protocol_id

uint8[<=128] data
```

**Current Usage**:
- **FC Side (uavcan driver)**: Subscribes to `gps_inject_data` uORB topic and publishes `RTCMStream` DroneCAN messages (`src/drivers/uavcan/sensors/gnss.cpp:588-616`)
- **CANnode Side**: Subscribes to `RTCMStream` and publishes to `gps_inject_data` uORB topic, which the GPS driver then writes to the GPS module (`src/drivers/uavcannode/Subscribers/RTCMStream.hpp`)

### 2. MovingBaselineData Message (Bidirectional RTCM for Moving Base)

**Location**: `src/drivers/uavcan/libdronecan/dsdl/ardupilot/gnss/20005.MovingBaselineData.uavcan`

```
uint8[<=300] data
```

**Key Characteristics**:
- Larger payload (300 bytes vs 128 bytes for RTCMStream)
- Used for Moving Base RTK configurations where RTCM flows bidirectionally
- Both publisher and subscriber implementations exist on both FC and CANnode sides

**FC Side**:
- `src/drivers/uavcan/sensors/gnss.cpp` - Publishes to MovingBaselineData from gps_inject_data
- Subscribes and converts incoming MovingBaselineData to gps_inject_data (for injection into GPS driver)

**CANnode Side**:
- `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` - Publishes gps_inject_data to DroneCAN
- `src/drivers/uavcannode/Subscribers/MovingBaselineData.hpp` - Subscribes and publishes to gps_inject_data

### 3. Reference Implementation: Septentrio SBF Tunnel (private_px4 repo)

The Septentrio X5 implementation in `/home/jake/code/ark/private_px4` provides an excellent reference for streaming raw GPS data via DroneCAN.

**Architecture**:
1. **CANnode GPS Driver** (`src/drivers/gnss/septentrio/septentrio_gps.cpp`):
   - Parses SBF messages from the GPS
   - Buffers specific message types (MeasEpoch, SatVisibility, GPSNav) in a circular buffer
   - Publishes chunks via `sbf_chunk_s` uORB topic

2. **CANnode UAVCAN Publisher** (`src/drivers/uavcannode/Publishers/SeptentrioSbfChunk.hpp`):
   - Subscribes to `sbf_chunk` uORB topic
   - Broadcasts via `uavcan::tunnel::Broadcast` DroneCAN message

3. **FC UAVCAN Subscriber** (`src/drivers/uavcan/sensors/septentrio_sbf_tunnel.cpp`):
   - Subscribes to `uavcan::tunnel::Broadcast`
   - Reassembles the stream and validates CRC
   - Could publish to uORB for logging

**Key Design Decisions in Septentrio Implementation**:
- Uses the existing `uavcan::tunnel::Broadcast` message (ID 2010)
- Protocol field set to `GPS_GENERIC` (value 2)
- Max payload of 60 bytes per chunk
- uORB queue length of 64 for the intermediate `sbf_chunk` topic
- Circular buffer on both CANnode (for buffering) and FC (for reassembly)

### 4. UAVCAN Tunnel Message

**Location**: `src/drivers/uavcan/libdronecan/dsdl/uavcan/tunnel/2010.Broadcast.uavcan`

```
Protocol protocol
uint8 channel_id
uint8[<=60] buffer    # TAO rules apply
```

**Protocol Definition** (`Protocol.uavcan`):
```
uint8 MAVLINK      = 0
uint8 MAVLINK2     = 1
uint8 GPS_GENERIC  = 2
uint8 UNDEFINED    = 255
uint8 protocol
```

### 5. GpsInjectData uORB Message

**Location**: `msg/GpsInjectData.msg`

```
uint64 timestamp
uint32 device_id
uint16 len
uint8 flags                     # LSB: 1=fragmented
uint8[300] data
uint8 ORB_QUEUE_LENGTH = 8
uint8 MAX_INSTANCES = 2
```

This is used bidirectionally - for RTCM corrections going to GPS and for RTCM observations coming from GPS.

---

## Implementation Options

### Option A: Create a New Dedicated PPKStream DroneCAN Message

**Pros**:
- Clean separation from RTK functionality
- Can be optimized specifically for PPK requirements
- Clear intent in the message name

**Cons**:
- Requires registering a new DSDL message type (data type ID allocation)
- More changes required

**Suggested Message Definition**:
```
# uavcan.equipment.gnss.PPKStream or similar
# Default data type ID: 106x (needs to be allocated)

uint8 PROTOCOL_ID_RTCM3 = 3
uint8 protocol_id

uint8[<=300] data  # Match MovingBaselineData capacity
```

### Option B: Reuse MovingBaselineData Message

**Pros**:
- Already exists and supports bidirectional RTCM
- 300-byte payload is suitable for RTCM messages
- Infrastructure already in place on both FC and CANnode

**Cons**:
- Semantically different purpose (Moving Base vs PPK)
- May cause confusion when debugging

**Implementation**:
- Add a new parameter to distinguish PPK output from Moving Base
- Reuse existing subscriber on FC side

### Option C: Use UAVCAN Tunnel Broadcast (Like Septentrio)

**Pros**:
- Proven approach (Septentrio implementation works)
- Generic, can be used for any data type
- No new DSDL message types needed

**Cons**:
- Smaller payload (60 bytes) requires more fragmentation
- Overhead of protocol/channel_id fields
- Need to implement stream reassembly on FC side

---

## Recommended Approach

Based on the research, I recommend **a hybrid approach**:

### Phase 1: CANnode Side (GPS → DroneCAN)

1. **Modify the GPS driver** to:
   - Configure the F9P/X20 to output the required RTCM messages (1005, 1077, 1087, 1097, 1127, 1230)
   - Parse the RTCM stream and extract complete messages
   - Publish RTCM data to a new `ppk_rtcm_data` uORB topic (or reuse `gps_inject_data` with a distinguishing flag)

2. **Create a new UAVCAN Publisher**:
   - Similar to `MovingBaselineDataPub`
   - Publishes to either:
     - A new `PPKStream` message (Option A), or
     - Existing `MovingBaselineData` with appropriate configuration (Option B)
   - Parameter-controlled enable/disable

### Phase 2: Flight Controller Side (DroneCAN → Logging)

1. **Create a new UAVCAN Subscriber** (or modify existing):
   - Subscribe to the PPK DroneCAN message
   - Publish to a new `ppk_rtcm_data` uORB topic

2. **Logger Integration**:
   - Ensure the PPK RTCM data is logged with appropriate timestamps
   - May need high-frequency logging configuration

---

## Key Files to Modify/Create

### CANnode Firmware:

1. `src/drivers/gps/devices/src/ubx.cpp` - Configure F9P RTCM output
2. `src/drivers/gps/gps.cpp` - Handle RTCM output stream
3. `src/drivers/uavcannode/Publishers/PPKStream.hpp` (new) - DroneCAN publisher
4. `src/drivers/uavcannode/UavcanNode.cpp` - Register new publisher
5. Board configs (e.g., `boards/ark/f9p-gps/default.px4board`) - Enable PPK feature

### Flight Controller Firmware:

1. `src/drivers/uavcan/sensors/ppk_rtcm.cpp` (new) - DroneCAN subscriber and bridge
2. `src/drivers/uavcan/uavcan_main.cpp` - Register new sensor bridge
3. `msg/PpkRtcmData.msg` (new, optional) - Dedicated uORB message for PPK data

### DroneCAN DSDL (if Option A):

1. `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/gnss/106x.PPKStream.uavcan` (new)

---

## F9P RTCM Output Configuration

The u-blox F9P supports outputting RTCM messages for PPK. Key configuration needed:

1. Enable RTCM3 output protocol on the serial port
2. Configure specific RTCM message rates:
   ```
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1005 = 1 (1Hz)
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1077 = 1 (1Hz)
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1087 = 1 (1Hz)
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1097 = 1 (1Hz)
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1127 = 1 (1Hz)
   UBX-CFG-MSGOUT-RTCM_3X_TYPE1230 = 1 (1Hz)
   ```

The existing code in `ubx.cpp` has infrastructure for RTCM output (see `OutputMode::GPSAndRTCM`), which can be leveraged.

---

## Data Rate Considerations

Estimated RTCM data rates:
- 1005: ~20 bytes @ 1Hz = 20 bytes/sec
- 1077 (GPS MSM7): ~200-500 bytes @ 1Hz = ~300 bytes/sec
- 1087 (GLONASS MSM7): ~200-400 bytes @ 1Hz = ~300 bytes/sec
- 1097 (Galileo MSM7): ~200-400 bytes @ 1Hz = ~300 bytes/sec
- 1127 (BeiDou MSM7): ~200-400 bytes @ 1Hz = ~300 bytes/sec
- 1230: ~50 bytes @ 1Hz = 50 bytes/sec

**Total**: ~1.3-1.8 KB/sec

This is manageable on CAN bus but requires:
- Adequate uORB queue depths
- Appropriate DroneCAN transfer priority
- Sufficient logger write speed

---

## Questions to Resolve Before Implementation

1. **DroneCAN Message Choice**: New PPKStream message vs reusing MovingBaselineData vs Tunnel?
2. **RTCM Message Rates**: Should all messages be at 1Hz or higher rates for better PPK accuracy?
3. **Logging Format**: How should PPK RTCM data be logged? Raw binary or parsed?
4. **Enable/Disable**: What parameter controls PPK output? Separate param or mode of existing param?
5. **Both Modules**: Same implementation for ARK X20 (ZED-F9P) and ARK RTK GPS (NEO-M8P)?

---

## References

- `src/drivers/uavcan/sensors/gnss.cpp` - GNSS UAVCAN bridge (FC side)
- `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` - Moving baseline publisher (CANnode)
- `src/drivers/uavcannode/Subscribers/RTCMStream.hpp` - RTCMStream subscriber (CANnode)
- `/home/jake/code/ark/private_px4/src/drivers/gnss/septentrio/` - Septentrio SBF reference
- `/home/jake/code/ark/private_px4/src/drivers/uavcannode/Publishers/SeptentrioSbfChunk.hpp` - Tunnel publisher
- `/home/jake/code/ark/private_px4/src/drivers/uavcan/sensors/septentrio_sbf_tunnel.cpp` - Tunnel receiver
