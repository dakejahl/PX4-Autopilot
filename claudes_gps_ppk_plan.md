# GPS PPK Implementation - RTCM Routing Separation

## Problem Statement

The current implementation has a flaw in how RTCM messages are routed. When `gotRTCMMessage` is called, it currently triggers both:
- `publishRTCMCorrections()` → publishes to `gps_inject_data` topic
- `publishPpkRtcmData()` → publishes to `ppk_rtcm_data` topic

This is incorrect because RTCM messages serve **different purposes** depending on the GPS mode:

| Mode | RTCM Purpose | Should publish to |
|------|--------------|-------------------|
| Moving Base | Corrections for Rover GPS | `gps_inject_data` only |
| PPK Mode | Raw observations for logging | `ppk_rtcm_data` only |
| RTK Base Station | Corrections for rovers | `gps_inject_data` only |

## Current State (What's Already Done)

1. **Parameter `GPS_UBX_MODE=7`** added for PPK mode in `src/drivers/gps/params.c`
2. **`_ppk_mode` flag** added to GPS class in `src/drivers/gps/gps.cpp`
3. **`GPS_DUMP_COMM`** cleaned up - removed RTCM option (value 2), now only 0=Disabled, 1=Full
4. **`publishPpkRtcmData()`** function implemented but currently only guards on `_ppk_mode`

## The Fix Required

### In `src/drivers/gps/gps.cpp`

The callback handler at line ~439 currently does:

```cpp
case GPSCallbackType::gotRTCMMessage:
    gps->publishRTCMCorrections((uint8_t *)data1, (size_t)data2);
    gps->publishPpkRtcmData((uint8_t *)data1, (size_t)data2);
    break;
```

This needs to be changed to route RTCM based on the mode. The logic should be:

```cpp
case GPSCallbackType::gotRTCMMessage:
    if (gps->isPpkMode()) {
        // PPK mode: RTCM is raw observation data for logging
        gps->publishPpkRtcmData((uint8_t *)data1, (size_t)data2);
    } else {
        // Moving Base / RTK Base: RTCM is corrections for other GPS units
        gps->publishRTCMCorrections((uint8_t *)data1, (size_t)data2);
    }
    break;
```

### Implementation Steps

1. **Add accessor method** to GPS class (or make callback a member function):
   ```cpp
   bool isPpkMode() const { return _ppk_mode; }
   ```

2. **Update the callback** to conditionally route RTCM messages based on mode

3. **Consider edge cases**:
   - What if someone wants both PPK logging AND moving base? (Probably not a valid use case)
   - The modes should be mutually exclusive

## Understanding the RTCM Flow

### When RTCM Parsing is Enabled (in `ubx.cpp:342`)

```cpp
if (_output_mode == OutputMode::GPSAndRTCM || _output_mode == OutputMode::RTCM || _mode == UBXMode::MovingBaseUART1) {
    if (!_rtcm_parsing) {
        _rtcm_parsing = new RTCMParsing();
    }
    _rtcm_parsing->reset();
}
```

RTCM parsing is enabled for:
- `OutputMode::GPSAndRTCM` - PPK mode (we set this when `_ppk_mode = true`)
- `OutputMode::RTCM` - Fixed base station mode
- `UBXMode::MovingBaseUART1` - Moving base sending corrections via UART1

### When RTCM Messages are Generated

In `ubx.cpp:1349-1356`:
```cpp
if (_rtcm_parsing) {
    if (_rtcm_parsing->addByte(b)) {
        gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
        decodeInit();
        _rtcm_parsing->reset();
        return ret;
    }
}
```

The `gotRTCMMessage()` callback is invoked for every complete RTCM message parsed.

## Mode Summary

| GPS_UBX_MODE | UBXMode | OutputMode | RTCM Destination |
|--------------|---------|------------|------------------|
| 0 (Default) | Normal | GPS | N/A (no RTCM output) |
| 1 (Heading Rover) | RoverWithMovingBase | GPS | N/A |
| 2 (Moving Base) | MovingBase | GPS | `gps_inject_data` |
| 3 (Heading UART1) | RoverWithMovingBaseUART1 | GPS | N/A |
| 4 (Moving Base UART1) | MovingBaseUART1 | GPS | `gps_inject_data` |
| 5 (Rover Static Base) | RoverWithStaticBaseUart2 | GPS | N/A |
| 6 (GCS) | GroundControlStation | GPS | N/A |
| 7 (PPK) | Normal | GPSAndRTCM | `ppk_rtcm_data` |

## Files to Modify

- `src/drivers/gps/gps.cpp` - Update callback routing logic

## Testing

1. **PPK Mode Test**: Set `GPS_UBX_MODE=7`, verify RTCM only goes to `ppk_rtcm_data`, not `gps_inject_data`
2. **Moving Base Test**: Set `GPS_UBX_MODE=2` or `4`, verify RTCM only goes to `gps_inject_data`, not `ppk_rtcm_data`
3. **Default Mode Test**: Set `GPS_UBX_MODE=0`, verify no RTCM messages are published anywhere

## Notes

- The `gps_inject_data` topic is used by the Secondary GPS (Rover) to receive corrections from the Primary GPS (Moving Base)
- The `ppk_rtcm_data` topic is logged for post-flight PPK processing
- These are fundamentally different use cases and should never overlap
