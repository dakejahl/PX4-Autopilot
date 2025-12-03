Overview: we are beginning to work on implementing PPK from the ARK X20 and ARK RTK GPS (X20 and F9P modules on these). This involves streaming over the relevant PPK messages over DroneCAN from the CANnode to the Flight Controller. Both firmwares use PX4, with one firmware being the normal PX4 Flight Controller firmware and the other firmware being a CANnode firmware.

---

This summary has been taken from an email chain:

The email thread discusses the requirement for Post-Processed Kinematics (PPK) support in the ARK X20 GPS unit for Deltaquad's product line, specifically for use with Auterion's Skynode running PX4.The key issue is that the current setup is ideal for RTK (PX4 publishes RTCM to ARK GPS), but PPK requires the opposite direction: the ARK X20 needs to publish its raw RTCM data over DroneCAN (ARK → PX4) so that PX4 can log it for post-processing.Relevant Details for PPK:
* Requirement: The ARK X20 firmware needs a new feature to stream RTCM data over DroneCAN. This is similar to a "Moving Base" configuration and also requires changes on the PX4 side to ensure fast sampling and logging of the data.
* RTCM Messages Required: Josip Pavlovic from Auterion confirmed the specific RTCM messages needed for their workflow:
   * 1005
   * 1077 (GPS MSM7)
   * 1087 (GLONASS MSM7)
   * 1097 (Galileo MSM7)
   * 1127 (BeiDou MSM7)
   * 1230 (Note: Kareem initially suggested MSM7 multi-constellation, and the list provided by Josip includes the corresponding MSM7 messages for four constellations plus 1005 and 1230).

---

We might need to create a new DroneCAN message similar to RTCMStream and MovingBaselineData, but dedicated to the PPK message set and workflow.

**RTCMStream**
Full name: uavcan.equipment.gnss.RTCMStream

Default data type ID: 1062

#
# GNSS RTCM SC-104 protocol raw stream container.
# RTCM messages that are longer than max data size can be split over multiple consecutive messages.
#
```
uint8 PROTOCOL_ID_UNKNOWN = 0
uint8 PROTOCOL_ID_RTCM2   = 2
uint8 PROTOCOL_ID_RTCM3   = 3
uint8 protocol_id

uint8[<=128] data
```

**MovingBaselineData**
Full name: ardupilot.gnss.MovingBaselineData

Default data type ID: 20005

# length of data is set per the number of bytes for pkt in
# libraries/AP_GPS/RTCM3_Parser.h
```
uint8[<=300] data
```

However what's weird about this is these messages are both just RTCM. I am thinking we might want to just replace these two messages with something universal:

**RTCM**
Full name: ardupilot.gnss.RTCM3

Default data type ID: ??
```
uint8 PROTOCOL_ID_UNKNOWN = 0
uint8 protocol_id
# TODO: what would we call these options? The cases are:
# 1. Dual GPS - Moving Base + Rover. Moving Base outputs MSM7 for Rover RTCM corrections for RTK and RTK heading and Flight Controller logging for PPK.
# 2. Dual GPS - Rover + Rover. Only one should output MSM7 for PPK.
# 3. Dual GPS - Rover + Rover. Static Base for RTK, both Rovers receiver RTCM corrections for RTK. Only one outputs MSM7 for PPK.
# 4. Single GPS - Rover. Groundside Static Base. Rover outputs MSM7 for PPK.
# 4. Single GPS - Rover. Groundside Static Base. Rover receives RTCM corrections for RTK. Rover outputs MSM7 for PPK.

uint8[<=300] data
```

---

I've done some similar work for our Septentrio X5 module, to send over raw SBF messages via the uavcan.tunnel message. I currently have that feature branch checked out at /home/jake/code/ark/private_px4 and you can navigate over there to take a look at the implmentation, the changes to check that matter are found in src/ and msg/.


---

We will begin this task by first doing some research. I'd like you to organize your findings into a new file called claudes_research.md. Based on this research we'll come up with an implementation plan. At this stage we are not yet writing any code.
