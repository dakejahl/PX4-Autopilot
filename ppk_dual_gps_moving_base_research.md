# Dual GPS Heading + PPK Logging Research

## Problem Statement

Currently, PX4's `GPS_UBX_MODE` parameter treats heading modes (1-4) and PPK mode (7) as mutually exclusive. However, users may want both:
- **Dual GPS heading** for real-time yaw estimation
- **PPK logging** for centimeter-accurate post-processed positioning

This would provide the best of both worlds: real-time heading during flight AND high-precision trajectory reconstruction after flight.

## Current Architecture

### Dual GPS Heading (Modes 1-4)
- **Moving Base**: Outputs RTCM observations → sent to Rover
- **Rover**: Receives RTCM from Moving Base → computes heading solution

### PPK Mode (Mode 7)
- **Single GPS**: Outputs RTCM raw observations → logged for post-processing

### The Question
Can one of the GPS units in a heading setup ALSO output raw observations for PPK logging? Two options:
1. **Rover + PPK**: The Rover outputs its raw observations for logging
2. **Moving Base + PPK**: The Moving Base outputs its raw observations for logging

## Technical Considerations

### Which GPS should output PPK data?

| Option | Pros | Cons |
|--------|------|------|
| Rover + PPK | Rover is computing the navigation solution; its observations are most relevant | Rover is already receiving RTCM from Moving Base; may have port conflicts |
| Moving Base + PPK | Moving Base is already outputting RTCM; may be easier to add PPK | Moving Base observations may be less useful for rover trajectory PPK |

### F9P Port/Output Considerations
- F9P has UART1, UART2, USB, I2C, SPI
- Different outputs can be configured on different ports
- Question: Can F9P output RTCM to Rover (UART2) AND output RTCM for logging (UART1) simultaneously?

## Investigation Steps

### Step 1: Research u-blox F9P capabilities
- [ ] Search for: "u-blox F9P moving base PPK simultaneously"
- [ ] Search for: "ZED-F9P RTCM output multiple ports"
- [ ] Check u-blox application notes for moving base + raw logging setups
- [ ] Review F9P interface description for simultaneous RTCM output capabilities

### Step 2: Research existing implementations
- [ ] Search for: "PX4 dual GPS heading PPK"
- [ ] Search for: "ArduPilot moving base PPK logging"
- [ ] Search for: "RTK heading PPK backup drone"
- [ ] Check Emlid, SwiftNav, or other RTK vendors for similar setups

### Step 3: Analyze PX4 driver configuration
- [ ] Review how `ubx.cpp` configures RTCM output for different modes
- [ ] Determine if Moving Base mode configures RTCM on UART2 only or also UART1
- [ ] Check if Rover mode can be combined with raw observation output
- [ ] Understand OutputMode enum and how it maps to actual F9P configuration

### Step 4: Determine implementation approach
Based on research, decide:
1. Is Rover + PPK or Moving Base + PPK more feasible?
2. Should this be a new mode (e.g., Mode 8, 9) or a separate parameter?
3. What F9P configuration changes are needed?

## Implementation Options (Pending Research)

### Option A: New Combined Modes
```
GPS_UBX_MODE:
  8 = Heading Rover + PPK (UART1 only setup)
  9 = Heading Moving Base + PPK (UART1 only setup)
```

### Option B: Separate PPK Parameter
```
GPS_UBX_MODE: 0-6 (existing modes)
GPS_PPK_ENABLE: 0 or 1 (orthogonal to mode, enables raw RTCM logging)
```

### Option C: Bitmask Parameter
```
GPS_UBX_CFG:
  Bit 0-2: Mode (0=Default, 1=Heading Rover, 2=Moving Base, etc.)
  Bit 3: PPK enable
```

## Notes

- For PPK post-processing, you typically need rover observations + base station observations
- The "base station" for PPK could be a CORS network or a ground-based static receiver (not the moving base on the vehicle)
- The moving base on the vehicle is NOT a suitable PPK base - it's moving!
- Therefore, PPK on the Rover makes more sense: log rover observations, pair with ground base for post-processing

## References

- [ZED-F9P Integration Manual](https://www.u-blox.com/en/docs/UBX-18010854)
- [ZED-F9P Moving Base Application Note](https://content.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf)
- PX4 GPS driver: `src/drivers/gps/`
- u-blox driver: `src/drivers/gps/devices/src/ubx.cpp`
