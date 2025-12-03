# PPK Implementation - Remaining Research Tasks

## Overview

This document lists the research tasks that need to be completed before or during implementation of PPK support. These items were identified during planning and require additional investigation.

---

## Research Task 1: PPK Message Update Rate

**Question**: What update rate is necessary or standard for PPK workflows?

**Background**:
- We're starting with 1Hz as the default
- PPK accuracy may benefit from higher rates (5Hz, 10Hz)
- Higher rates increase CAN bus load and logging storage requirements

**Research Actions**:
1. Look up PPK post-processing tool documentation (e.g., RTKLIB, Emlid Studio, etc.)
2. Check what rates are typically used in surveying/mapping workflows
3. Review u-blox F9P documentation for recommended RTCM output rates
4. Consider the trade-off between accuracy and data volume

**Output**: Recommended message rate with justification

---

## Research Task 2: PPK Data Format for Post-Processing

**Question**: Do PPK post-processing tools consume raw RTCM data, or is additional processing/formatting needed?

**Background**:
- We plan to log raw RTCM data
- Post-processing tools may expect specific file formats (e.g., RINEX, native RTCM3 files)
- Understanding the tool chain will inform logging approach

**Research Actions**:
1. Investigate common PPK post-processing tools and their input formats
2. Check if tools can directly ingest RTCM3 streams
3. Look for any existing PX4 or drone-based PPK workflows
4. Determine if timestamp requirements affect how data should be logged

**Output**: Confirmation that raw RTCM logging is sufficient, or recommendations for alternative formats

---

## Research Task 3: RTCM Message Size Analysis

**Question**: What are the actual sizes of the RTCM messages we need to transmit?

**Background**:
- Initial estimates from research: 1.3-1.8 KB/sec total
- DroneCAN payload size affects fragmentation strategy
- Need actual measurements to inform buffer sizes and queue depths

**Research Actions**:
1. Find official RTCM 10403.x documentation or reliable secondary sources for message sizes
2. Analyze actual F9P RTCM output if a device is available
3. Determine maximum single message size for buffer allocation

**Output**: Table of RTCM message types with min/max/typical sizes

---

## Research Task 4: DroneCAN Message ID Allocation

**Question**: What data type ID should we use for the PPKStream message?

**Background**:
- RTCMStream uses 1062
- Need to pick an unused ID in the appropriate range
- Should follow DroneCAN conventions

**Research Actions**:
1. Review existing DSDL files in the gnss namespace to identify used IDs
2. Check DroneCAN/UAVCAN specification for ID allocation guidelines
3. Determine if there's a registration process or if we can pick a "vendor" range ID

**Output**: Selected data type ID with justification

---

## Research Task 5: Parameter Naming Convention

**Question**: What should the PPK enable parameter be named?

**Background**:
- Needs to follow PX4 parameter naming conventions
- Should be clear and discoverable
- May need to consider interaction with existing GPS/RTCM parameters

**Research Actions**:
1. Review existing GPS-related parameters in PX4
2. Check parameter naming conventions in the PX4 documentation
3. Propose parameter name that fits established patterns

**Output**: Proposed parameter name (e.g., `GPS_PPK_EN`, `GPS_PPK_OUTPUT`, etc.)

---

## Research Task 6: Logger Integration Details

**Question**: How should PPK RTCM data be integrated with the PX4 logger?

**Background**:
- Need to ensure data is logged with appropriate timestamps
- Raw RTCM may need special handling (binary data in uORB message)
- Log file extraction workflow needs to be considered

**Research Actions**:
1. Review how other binary/raw data is logged in PX4 (e.g., GPS raw data)
2. Check logger configuration for adding new topics
3. Investigate log extraction tools and how raw data can be retrieved

**Output**: Recommended logging approach and any logger configuration changes needed

---

## Priority Order

1. **Task 4** (DroneCAN ID) - Needed for Part 1 of implementation
2. **Task 5** (Parameter naming) - Needed for Part 2 of implementation
3. **Task 3** (Message sizes) - Informs buffer sizing in Parts 3-5
4. **Task 1** (Update rate) - Informs configuration in Part 2
5. **Task 2** (Data format) - Informs logging in Part 6
6. **Task 6** (Logger details) - Needed for Part 6
