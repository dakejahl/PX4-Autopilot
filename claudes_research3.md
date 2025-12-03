# PPK Implementation - Research Results

## Task 4: DroneCAN Message ID Allocation

**Decision**: Use ID **1064** as `uavcan.equipment.gnss.PPKStream`

**Why**: Sequential with existing GNSS messages (1060-1063), logically grouped with RTCMStream (1062). Both are RTCM streams, just different directions.

---

## Task 5: Parameter Naming Convention

**Decision**: Extend `GPS_UBX_MODE` with value 7 for PPK

**Why**: `GPS_UBX_MODE` already controls RTCM-related configurations (modes 1-6 handle heading, moving base, rover setups). PPK is another RTCM output configuration, so it fits naturally as mode 7.

---

## Task 3: RTCM Message Size Analysis

**Decision**: Use 300-byte payload (matching MovingBaselineData), expect ~1.5-2 KB/sec total throughput at 1Hz.

**Why**: MSM7 messages (1077, 1087, 1097, 1127) are ~200-450 bytes each depending on satellite count. 128 bytes (RTCMStream size) would require frequent fragmentation. 300 bytes handles most messages in a single transfer.

---

## Task 1: PPK Message Update Rate

**Decision**: Default to 5Hz, with option for 1Hz or 10Hz via parameter if needed later.

**Why**: Drone PPK workflows typically use 5-10Hz. At drone speeds of 20 m/s, 5Hz gives position every 4m which is adequate for most photogrammetry. 1Hz is too coarse for moving platforms; 10Hz is optimal but doubles bandwidth.

---

## Task 2: PPK Data Format for Post-Processing

**Decision**: Log raw RTCM3 data. Post-processing tools (RTKLIB, Emlid Studio) convert RTCM3 to RINEX automatically.

**Why**: RTCM3 is a standard input format. Tools like RTKCONV and Emlid Studio handle RTCM3→RINEX conversion natively. No need to pre-convert on the flight controller.

---

## Task 6: Logger Integration Details

**Decision**: Create new `PpkRtcmData.msg` uORB topic similar to `GpsDump.msg`, add to logger as optional topic.

**Why**: `GpsDump.msg` pattern already exists (79-byte chunks, timestamp, instance). PPK needs larger chunks (~300 bytes) to match DroneCAN payload. Logger already handles `gps_dump` as optional topic - same pattern works for PPK.
