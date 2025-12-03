# PPK + Dual GPS Heading Investigation Report

## Executive Summary

This investigation confirms that enabling PPK (Post-Processed Kinematics) alongside dual GPS heading is both **technically feasible** and **desirable**. The implementation adds a **separate `GPS_UBX_PPK` parameter** rather than creating new combined modes.

**Key Finding**: The Rover (not Moving Base) should output PPK data, and this can be done independently of the heading mode configuration.

**Implementation Status**: Complete - added `GPS_UBX_PPK` parameter and removed mode 7 from `GPS_UBX_MODE`.

---

## Background

### Current Architecture

PX4's `GPS_UBX_MODE` parameter treats heading modes (1-4) and PPK mode (7) as mutually exclusive:

| Mode | Description | UBXMode Enum |
|------|-------------|--------------|
| 0 | Default | Normal |
| 1 | Heading Rover (UART2) | RoverWithMovingBase |
| 2 | Heading Moving Base (UART2) | MovingBase |
| 3 | Heading Rover (UART1) | RoverWithMovingBaseUART1 |
| 4 | Heading Moving Base (UART1) | MovingBaseUART1 |
| 5 | RTK Rover (static base UART2) | RoverWithStaticBaseUart2 |
| 6 | GCS (NMEA on UART2) | GroundControlStation |

Additionally, `GPS_UBX_PPK` enables PPK output orthogonally to the mode setting.

### The Problem (Solved)

Users want **both**:
1. **Dual GPS heading** for real-time yaw estimation
2. **PPK logging** for centimeter-accurate post-processed positioning

This is now supported by setting both `GPS_UBX_MODE` (for heading) and `GPS_UBX_PPK=1` (for PPK).

---

## Technical Analysis

### 1. u-blox F9P Capabilities

Research confirms the F9P can handle simultaneous operations:

- **Multiple Ports**: F9P has UART1, UART2, USB, I2C, SPI - different outputs can be configured on different ports simultaneously
- **Simultaneous Logging**: "Users can log raw data for PPK at the same time as using the realtime solution offered by the F9P"
- **Moving Base + Raw Data**: The Moving Base outputs RTCM on UART2 to Rover, while raw observations can be output on UART1 for logging

**Sources**:
- [u-blox ZED-F9P Moving Base Application Note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf)
- [rtklibexplorer: PPK with F9P](https://rtklibexplorer.wordpress.com/2019/08/24/dual-frequency-ppk-solutions-with-rtklib-and-the-u-blox-f9p/)

### 2. Which GPS Should Output PPK Data?

**Answer: The Rover**

| Criterion | Rover | Moving Base |
|-----------|-------|-------------|
| Navigation solution | Computes the heading/position | Only provides corrections |
| PPK relevance | Its observations define the trajectory | Observations not useful for rover trajectory |
| Base station requirement | Pairs with external CORS/base | Would need separate base for itself |
| Current implementation | Already the "main" GPS in PX4 | Secondary GPS |

**Critical Insight**: For PPK post-processing, you need:
1. **Rover raw observations** (the trajectory you want to refine)
2. **External base station data** (CORS network or ground-based static receiver)

The Moving Base on the vehicle is **not suitable** as a PPK base - it's moving! The Moving Base only provides short-baseline RTCM corrections for heading computation, not for PPK reference.

### 3. RTCM Message Types Analysis

#### Moving Base Mode (Heading)
Uses MSM4 messages for compact size:
- 1074 (GPS), 1084 (GLONASS), 1094 (Galileo), 1124 (BeiDou)
- 1230 (GLONASS biases)
- 4072 (u-blox proprietary for moving base)

#### PPK Mode
Uses MSM7 messages for higher precision + Doppler:
- 1077 (GPS), 1087 (GLONASS), 1097 (Galileo), 1127 (BeiDou)
- 1005 (Base station ARP)
- 1230 (GLONASS biases)

**Key Difference**: MSM7 (PPK) provides higher resolution and includes Doppler measurements essential for kinematic processing. MSM4 (Moving Base) is sufficient for short-baseline RTK but lacks the precision needed for PPK.

Sources:
- [RTCM Message Cheat Sheet](https://www.use-snip.com/kb/knowledge-base/an-rtcm-message-cheat-sheet/)
- [MSM4 vs MSM7 Comparison](https://community.gpswebshop.com/forum/topic/notes-for-msm4-vs-msm7-in-the-rtcm-configuration-of-the-ex-1/)

### 4. Current PX4 Implementation Analysis

#### Code Flow for PPK Mode (Mode 7)
```
gps.cpp:801-806:
  case 7:  // PPK mode
    if (_instance == Instance::Main) {
      _ppk_mode = true;  // Only sets flag, doesn't change ubx_mode
    }
    break;

gps.cpp:943-948:
  if (_ppk_mode) {
    gpsConfig.output_mode = GPSHelper::OutputMode::GPSAndRTCM;
  } else {
    gpsConfig.output_mode = GPSHelper::OutputMode::GPS;
  }
```

#### Code Flow for Heading Modes (Mode 1/3)
```
gps.cpp:765-772:
  case 1:  // heading
    if (_instance == Instance::Main) {
      ubx_mode = GPSDriverUBX::UBXMode::RoverWithMovingBase;
    } else {
      ubx_mode = GPSDriverUBX::UBXMode::MovingBase;
    }
    break;
```

#### Key Observation
The `_ppk_mode` flag and the heading `UBXMode` are **orthogonal concepts**:
- `_ppk_mode` controls whether RTCM observations are published to `ppk_rtcm_data` topic
- `UBXMode` controls UART2 configuration for receiving/sending RTCM to another F9P

These can coexist! A Rover can:
1. Receive RTCM from Moving Base (for heading)
2. Output raw RTCM observations on UART1 (for PPK logging)

---

## Implementation Options Analysis

### Option A: New Combined Modes
```
GPS_UBX_MODE:
  8 = Heading Rover + PPK (UART2)
  9 = Heading Rover + PPK (UART1)
```

**Pros**: Single parameter
**Cons**:
- Combinatorial explosion (would need 4 new modes for all rover variants)
- Confusing - PPK is not a "mode" but an "additional feature"
- Inconsistent with other GPS params

### Option B: Separate PPK Parameter (Recommended)

```
GPS_UBX_MODE: 0-6 (existing modes, remove mode 7)
GPS_UBX_PPK:  0 = Disabled, 1 = Enable PPK output on Rover
```

**Pros**:
- Clean separation of concerns
- Works with ANY rover mode (0, 1, 3, 5)
- Consistent with existing params (GPS_UBX_CFG_INTF, GPS_DUMP_COMM)
- No need to deprecate mode 7 - just document as "legacy, use GPS_UBX_PPK instead"
- Easier to extend (could add GPS_UBX_PPK=2 for Moving Base PPK if ever needed)

**Cons**:
- Two parameters instead of one (minor)

### Option C: Bitmask Parameter
```
GPS_UBX_CFG:
  Bit 0-2: Mode (0=Default, 1=Rover, 2=MB, etc.)
  Bit 3: PPK enable
```

**Pros**: Single parameter, extensible
**Cons**:
- Breaking change to GPS_UBX_MODE
- Less user-friendly
- Would require migration path

---

## Implementation (Completed)

**Added `GPS_UBX_PPK` parameter and removed mode 7 from `GPS_UBX_MODE`.**

### Parameter Definition (src/drivers/gps/params.c)

```c
/**
 * Enable PPK raw RTCM output
 *
 * When enabled, the GPS driver outputs raw RTCM observation messages
 * (MSM7 type) for Post-Processed Kinematics (PPK). These are published
 * to the ppk_rtcm_data topic and logged for post-processing with tools
 * like RTKLIB or Emlid Studio.
 *
 * PPK post-processing requires both rover observations (logged by PX4)
 * and base station data (from a CORS network or ground-based receiver).
 *
 * This parameter can be combined with GPS_UBX_MODE heading modes (1, 3)
 * to get both real-time GPS heading AND centimeter-accurate post-processed
 * positioning. When combined with heading modes, only the Rover GPS outputs
 * PPK data.
 *
 * @boolean
 * @reboot_required true
 * @group GPS
 */
PARAM_DEFINE_INT32(GPS_UBX_PPK, 0);
```

### Changes Made

1. **params.c**:
   - Added `GPS_UBX_PPK` parameter definition
   - Removed mode 7 from `GPS_UBX_MODE` (max now 6)
   - Updated `GPS_DUMP_COMM` comment to reference `GPS_UBX_PPK`

2. **gps.cpp**:
   - Removed mode 7 case from switch statement
   - Added code to read `GPS_UBX_PPK` parameter after mode parsing
   - Set `_ppk_mode = true` when `GPS_UBX_PPK == 1` AND `_instance == Instance::Main`
   - Updated comment on `_ppk_mode` member variable

### Example Configurations

| Use Case | GPS_UBX_MODE | GPS_UBX_PPK | Result |
|----------|--------------|-------------|--------|
| Heading only | 1 | 0 | Dual GPS heading, no PPK |
| PPK only | 0 | 1 | Single GPS with PPK logging |
| Heading + PPK | 1 | 1 | Dual GPS heading + PPK on Rover |
| Heading (UART1) + PPK | 3 | 1 | Dual GPS heading via UART1 + PPK on Rover |

---

## UAVCAN Considerations

The existing UAVCAN PPK infrastructure already supports this:

1. **CAN GPS Node** (uavcannode):
   - Already has `PPKStreamPub` publisher (`src/drivers/uavcannode/Publishers/PPKStream.hpp`)
   - Subscribes to `ppk_rtcm_data` and broadcasts over CAN

2. **Autopilot** (uavcan driver):
   - Already has `UavcanPpkStreamBridge` subscriber (`src/drivers/uavcan/sensors/ppk_stream.cpp`)
   - Receives PPK data from CAN and publishes to `ppk_rtcm_data`

3. **New Parameter Needed**: Add `CANNODE_PUB_PPK` to enable PPK publication from CAN node
   - Similar to existing `CANNODE_PUB_MBD` for Moving Base Data

---

## Conclusion

Adding `GPS_UBX_PPK` as a separate parameter is the cleanest solution because:

1. **Conceptually correct**: PPK is an optional feature, not a mode
2. **Flexible**: Works with any rover configuration
3. **Non-breaking**: Preserves mode 7 for backwards compatibility
4. **Extensible**: Easy to add more PPK options in the future
5. **UAVCAN-ready**: Infrastructure already exists

The Rover should output PPK data because its observations define the trajectory being refined. The Moving Base's observations are not useful for PPK since the Moving Base is also moving and cannot serve as a reference station.

---

## References

- [ZED-F9P Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
- [ZED-F9P Moving Base Application Note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf)
- [rtklibexplorer: Dual-frequency PPK with F9P](https://rtklibexplorer.wordpress.com/2019/08/24/dual-frequency-ppk-solutions-with-rtklib-and-the-u-blox-f9p/)
- [ArduSimple: How to configure ZED-F9P](https://www.ardusimple.com/how-to-configure-ublox-zed-f9p/)
- [RTCM Message Cheat Sheet](https://www.use-snip.com/kb/knowledge-base/an-rtcm-message-cheat-sheet/)
- [Propeller: How PPK Drone Surveying Works](https://www.propelleraero.com/blog/how-ppk-drone-surveying-works/)

---

## Appendix: Code Locations

| File | Purpose |
|------|---------|
| `src/drivers/gps/params.c:120` | GPS_UBX_MODE definition |
| `src/drivers/gps/params.c:122-142` | GPS_UBX_PPK definition |
| `src/drivers/gps/gps.cpp:218` | `_ppk_mode` member variable |
| `src/drivers/gps/gps.cpp:807-817` | GPS_UBX_PPK parameter handling |
| `src/drivers/gps/gps.cpp:943-948` | PPK mode output config |
| `src/drivers/gps/gps.cpp:439-449` | RTCM callback routing |
| `src/drivers/gps/gps.cpp:1347-1376` | publishPpkRtcmData() |
| `src/drivers/gps/devices/src/ubx.cpp:342-348` | RTCMParsing init |
| `src/drivers/gps/devices/src/ubx.cpp:953-985` | MovingBase UART2 config |
| `src/drivers/gps/devices/src/ubx.cpp:2575-2638` | activateRTCMOutput() |
| `msg/PpkRtcmData.msg` | PPK uORB message |
