The Mosaic-X5 module can be configured to output the GNSS observables

The driver will need to be configured to set the module to output them, then read into a custom uorb message, and publish to the dronecan tunnel/2010.Broadcast.uavcan message.

In addition, we will need the decoded GPS navigation message (GPSNav) which has the information about the GPS constellation clock and orbit.

might want to fetch the SatVisibility message which contains the azimuth and elevation of the satellites

need GPSNav (281) which has the information about the GPS constellation clock and orbit.


### Requirements

MeasEpoch (pg 252)
```cpp
typedef struct {
    int8_t   Sync1;
    int8_t   Sync2;
    uint16_t CRC;              // Block Header, see 4.1.1
    uint16_t ID;
    uint16_t Length;           // 1 byte
    uint32_t TOW;              // 0.001 s, Do-Not-Use: 4294967295, Receiver time stamp, see 4.1.3
    uint16_t WNc;              // 1 week, Do-Not-Use: 65535
    uint8_t  N1;               // Number of MeasEpochChannelType1 sub-blocks in this MeasEpoch block
    uint8_t  SB1Length;        // 1 byte, Length of a MeasEpochChannelType1 sub-block, excluding the nested MeasEpochChannelType2 sub-blocks
    uint8_t  SB2Length;        // 1 byte, Length of a MeasEpochChannelType2 sub-block
    uint8_t  CommonFlags;      // Bit field containing flags common to all measurements
                               // Bit 0: Multipath mitigation
                               // Bit 1: Smoothing (see setSmoothingInterval command)
                               // Bit 2: Reserved
                               // Bit 3: Clock steering (see setClockSyncThreshold command)
                               // Bit 4: Not applicable
                               // Bit 5: High dynamics (setReceiverDynamics command)
                               // Bit 6: E6B signal (Galileo E6 measurements)
                               // Bit 7: Scrambling bit
    uint8_t  CumClkJumps;      // 0.001 s, Cumulative millisecond jumps since start-up
    uint8_t  Reserved;         // Reserved for future use, to be ignored by decoding software

    // Variable length data follows:
    // Type1: A succession of N1 MeasEpochChannelType1 sub-blocks (see definition below)
    // MeasEpochChannelType1_t Type1[N1];  // Placeholder - actual implementation varies

    // Padding bytes, see 4.1.5
    // uint8_t Padding[];
} MeasEpoch_t;
```

MeasEpochChannelType1 (pg 253)
```cpp
typedef struct {
    uint8_t  RxChannel;        // Receiver channel on which this satellite is currently tracked (see 4.1.11)
    uint8_t  Type;             // Bit field indicating the signal type and antenna ID:
                               // Bits 0-4: SigIdxLo: if not 31, this is the signal number (see 4.1.10),
                               //           otherwise the signal number can be found in the ObsInfo field below
                               // Bits 5-7: Antenna ID: 0 for main, 1 for Aux1 and 2 for Aux2
    uint8_t  SVID;             // Satellite ID, see 4.1.9
    uint8_t  Misc;             // Bit field containing the MSB of the pseudorange
                               // Bits 0-3: CodeMSB: MSB of the pseudorange (this is an unsigned value)
                               // Bits 4-7: Reserved
                               // Do-Not-Use: 4294967.296 m
    uint32_t CodeLSB;          // 0.001 m, Do-Not-Use: 0, LSB of the pseudorange
                               // The pseudorange expressed in meters is computed as follows:
                               // PR_type1[m] = (CodeMSB*4294967296+CodeLSB)*0.001
                               // where CodeMSB is part of the Misc field
    int32_t  Doppler;          // 0.0001 Hz, Do-Not-Use: -2147483648
                               // Carrier Doppler (positive for approaching satellites)
                               // To compute the Doppler in Hz, use: D_type1[Hz] = Doppler*0.0001
    uint16_t CarrierLSB;       // 0.001 cycles, Do-Not-Use: 0
                               // LSB of the carrier phase relative to the pseudorange
    int8_t   CarrierMSB;       // 65.536 cycles, Do-Not-Use: -128
                               // MSB of the carrier phase relative to the pseudorange
                               // The full carrier phase can be computed by:
                               // L[cycles] = PR_type1[m]/λ + (CarrierMSB*65536+CarrierLSB)*0.001
                               // where λ is the carrier wavelength corresponding to the frequency
                               // of the signal type in the Type field above:
                               // λ=299792458/f_L m, with f_L the carrier frequency as listed in section 4.1.10
    uint8_t  CN0;              // 0.25 dB-Hz, Do-Not-Use: 255
                               // The C/N0 in dB-Hz is computed as follows, depending on the signal type in the Type field:
                               // C/N0[dB-Hz] = CN0*0.25 if the signal number is 1 or 2
                               // C/N0[dB-Hz] = CN0*0.25+10 otherwise
                               // Users requiring a higher C/N0 resolution can use the MeasExtra_sub block
    uint16_t LockTime;         // 1 s, Do-Not-Use: 65535
                               // Duration of continuous carrier phase. The lock-time is reset at the initial lock
                               // of the phase-locked-loop, and whenever a loss of lock condition occurs.
                               // If the lock-time is longer than 65534s, it is clipped to 65534s.
                               // If the carrier phase measurement is not available, this field is set to its Do-Not-Use value
    uint8_t  ObsInfo;          // Bit field:
                               // Bit 0: if set, the pseudorange measurement is smoothed
                               // Bit 1: Reserved
                               // Bit 2: this bit is set when the carrier phase (L) has a half-cycle ambiguity
                               // Bits 3-7: The interpretation of these bits depends on the value of SigIdxLo from the Type field
                               //           If SigIdxLo equals 31, these bits contain the signal number with an offset of 32 (see 4.1.10)
                               //           If SigIdxLo is 8, 9, 10 or 11, these bits contain the GLONASS frequency number with an offset of 8
                               //           Otherwise, these bits are reserved
    uint8_t  N2;               // Number of MeasEpochChannelType2 sub-blocks contained in this MeasEpochChannelType1 sub-block

    // Variable length data follows:
    // uint8_t Padding[];      // Padding bytes, see 4.1.5
    // Type2: A succession of N2 MeasEpochChannelType2 sub-blocks (see definition below)
    // MeasEpochChannelType2_t Type2[N2];  // Placeholder - actual implementation varies
} MeasEpochChannelType1_t;
```

MeasEpochChannelType2 (pg 254)
```cpp
typedef struct {
    uint8_t  Type;             // Bit field indicating the signal type and antenna ID:
                               // Bits 0-4: SigIdxLo: if not 31, this is the signal number (see 4.1.10),
                               //           otherwise the signal number can be found in the ObsInfo field below
                               // Bits 5-7: Antenna ID: 0 for main, 1 for Aux1 and 2 for Aux2
    uint8_t  LockTime;         // 1 s, Do-Not-Use: 255
                               // See corresponding field in the MeasEpochChannelType1 sub-block above,
                               // except that the value is clipped to 254 instead of 65534
    uint8_t  CN0;              // 0.25 dB-Hz, Do-Not-Use: 255
                               // See corresponding field in the MeasEpochChannelType1 sub-block above
    uint8_t  OffsetsMSB;       // Bit field containing the MSB of the code and of the Doppler offsets
                               // with respect to the MeasEpochChannelType1 sub-block
                               // Bits 0-2: CodeOffsetMSB: MSB of the code offset (65.536 m, Do-Not-Use: -4)
                               // Bits 3-7: DopplerOffsetMSB: MSB of the Doppler offset (6.5536 Hz, Do-Not-Use: -16)
                               // CodeOffsetMSB and DopplerOffsetMSB are coded as two's complement
                               // Refer to the CodeOffsetLSB and DopplerOffsetLSB fields to see how to use this field
    int8_t   CarrierMSB;       // 65.536 cycles, Do-Not-Use: -128
                               // MSB of the carrier phase relative to the pseudorange
    uint8_t  ObsInfo;          // Bit field:
                               // Bit 0: if set, the pseudorange measurement is smoothed
                               // Bit 1: Reserved
                               // Bit 2: this bit is set when the carrier phase (L) has a half-cycle ambiguity
                               // Bits 3-7: If SigIdxLo from the Type field of this sub-block equals 31,
                               //           these bits contain the signal number with an offset of 32 (see 4.1.10),
                               //           e.g. 1 corresponds to signal number 33 (QZSS L1S).
                               //           Otherwise they are reserved and must be ignored by the decoding software
    uint16_t CodeOffsetLSB;    // 0.001 m, Do-Not-Use: 0
                               // LSB of the code offset with respect to pseudorange in the MeasEpochChannelType1 sub-block
                               // To compute the pseudorange, use:
                               // PR_type2[m] = PR_type1[m]
                               //               + (CodeOffsetMSB*65536+CodeOffsetLSB)*0.001
    uint16_t CarrierLSB;       // 0.001 cycles, Do-Not-Use: 0
                               // LSB of the carrier phase relative to the pseudorange
                               // The full carrier phase can be computed by:
                               // L[cycles]= PR_type2[m]/λ
                               //            +(CarrierMSB*65536+CarrierLSB)*0.001
                               // where λ is the carrier wavelength corresponding to the signal type in the Type field
    uint16_t DopplerOffsetLSB; // 0.0001 Hz, Do-Not-Use: 0
                               // LSB of the Doppler offset relative to the Doppler in the MeasEpochChannelType1 sub-block
                               // To compute the Doppler, use:
                               // D_type2[Hz] = D_type1[Hz]*α
                               //               +(DopplerOffsetMSB*65536+DopplerOffsetLSB)*1e-4,
                               // where α is the ratio of the carrier frequency corresponding to the observable type
                               // in this MeasEpochChannelType2 sub-block, and that of the master observable type
                               // in the parent MeasEpochChannelType1 sub-block (see section 4.1.10 for a list of all carrier frequencies)
    // Note: Padding bytes would follow based on the parent block structure, see 4.1.5
} MeasEpochChannelType2_t;
```

SatVisibility (pg 388)
```cpp
typedef struct {
    int8_t   Sync1;
    int8_t   Sync2;
    uint16_t CRC;              // Block Header, see 4.1.1
    uint16_t ID;
    uint16_t Length;           // 1 byte
    uint32_t TOW;              // 0.001 s, Do-Not-Use: 4294967295, Receiver time stamp, see 4.1.3
    uint16_t WNc;              // 1 week, Do-Not-Use: 65535
    uint8_t  N;                // Number of satellites for which information is provided in this SBF block,
                               // i.e. number of SatInfo sub-blocks
    uint8_t  SBLength;         // 1 byte, Length of one SatInfo sub-block

    // Variable length data follows:
    // SatInfo: A succession of N SatInfo sub-blocks, see definition below
    // SatInfo_t SatInfo[N];   // Placeholder - actual implementation varies

    // Padding bytes, see 4.1.5
    // uint8_t Padding[];
} SatVisibility_t;
```

SatInfo (pg 388)
```cpp
typedef struct {
    uint8_t  SVID;             // Satellite ID, see 4.1.9
    uint8_t  FreqNr;           // Do-Not-Use: 0
                               // For GLONASS FDMA signals, this is the frequency number, with an offset of 8.
                               // It ranges from 1 (corresponding to an actual frequency number of -7) to 21
                               // (corresponding to an actual frequency number of 13).
                               // Otherwise, FreqNr is reserved and must be ignored by the decoding software.
    uint16_t Azimuth;          // 0.01 degrees, Do-Not-Use: 65535
                               // Azimuth. 0 is North, and azimuth increases towards East.
    int16_t  Elevation;        // 0.01 degrees, Do-Not-Use: -32768
                               // Elevation relative to local horizontal plane.
    uint8_t  RiseSet;          // Rise/set indicator:
                               // 0:   satellite setting
                               // 1:   satellite rising
                               // 255: elevation rate unknown
    uint8_t  SatelliteInfo;    // Satellite visibility info based on:
                               // 1:   almanac
                               // 2:   ephemeris
                               // 255: unknown

    // Padding bytes, see 4.1.5
    // uint8_t Padding[];
} SatInfo_t;
```

GPSNav (pg 281)
```cpp
typedef struct {
    int8_t   Sync1;
    int8_t   Sync2;
    uint16_t CRC;              // Block Header, see 4.1.1
    uint16_t ID;
    uint16_t Length;           // 1 byte
    uint32_t TOW;              // 0.001 s, Do-Not-Use: 4294967295, SIS time stamp, see 4.1.3
    uint16_t WNc;              // 1 week, Do-Not-Use: 65535
    uint8_t  PRN;              // ID of the GPS satellite of which the ephemeris is given in this block (see 4.1.9)
    uint8_t  Reserved;         // Reserved for future use, to be ignored by decoding software
    uint16_t WN;               // 1 week, Do-Not-Use: 65535, Week number (10 bits from subframe 1, word 3)
    uint8_t  CAorPonL2;        // Code(s) on L2 channel (2 bits from subframe 1, word 3)
    uint8_t  URA;              // User Range accuracy index (4 bits from subframe 1 word 3)
    uint8_t  health;           // 6-bit health from subframe 1, word 3 (6 bits from subframe 1, word 3)
    uint8_t  L2DataFlag;       // Data flag for L2 P-code (1 bit from subframe 1, word 4)
    uint16_t IODC;             // Issue of data, clock (10 bits from subframe 1)
    uint8_t  IODE2;            // Issue of data, ephemeris (8 bits from subframe 2)
    uint8_t  IODE3;            // Issue of data, ephemeris (8 bits from subframe 3)
    uint8_t  FitIntFlg;        // Curve Fit Interval, (1 bit from subframe 2, word 10)
    uint8_t  Reserved2;        // unused, to be ignored by decoding software
    float    T_gd;             // 1 s, Estimated group delay differential
    uint32_t t_oc;             // 1 s, clock data reference time
    float    a_f2;             // 1 s/s^2, SV clock aging
    float    a_f1;             // 1 s/s, SV clock drift
    float    a_f0;             // 1 s, SV clock bias
    float    C_rs;             // 1 m, Amplitude of the sine harmonic correction term to the orbit radius
    float    DEL_N;            // 1 semi-circle/s, Mean motion difference from computed value
    double   M_0;              // 1 semi-circle, Mean anomaly at reference time
    float    C_uc;             // 1 rad, Amplitude of the cosine harmonic correction term to the argument of latitude
    double   e;                // Eccentricity
    float    C_us;             // 1 rad, Amplitude of the sine harmonic correction term to the argument of latitude
    double   SQRT_A;           // 1 m^(1/2), Square root of the semi-major axis
    uint32_t t_oe;             // 1 s, Reference time ephemeris
    float    C_ic;             // 1 rad, Amplitude of the cosine harmonic correction term to the angle of inclination
    double   OMEGA_0;          // 1 semi-circle, Longitude of ascending node of orbit plane at weekly epoch
    float    C_is;             // 1 rad, Amplitude of the sine harmonic correction term to the angle of inclination
    double   i_0;              // 1 semi-circle, Inclination angle at reference time
    float    C_rc;             // 1 m, Amplitude of the cosine harmonic correction term to the orbit radius
    double   omega;            // 1 semi-circle, Argument of perigee
    float    OMEGADOT;         // 1 semi-circle/s, Rate of right ascension
    float    IDOT;             // 1 semi-circle/s, Rate of inclination angle
    uint16_t WNt_oc;           // 1 week, WN associated with t_oc, modulo 1024
    uint16_t WNt_oe;           // 1 week, WN associated with t_oe, modulo 1024
    uint8_t  Padding[];        // Padding bytes, see 4.1.5
} GPSNav_t;
```
