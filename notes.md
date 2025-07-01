## Rangefinder in PX4
The rangefinder distance (Range) can be used for 2 distinct purposes: EKF Z Position/Velocity observation, and Terrain Height observation. When the Range is used for EKF observation, the Position observation is relative

### Definitions
Altitude = EKF Z Position
Range = Distance to ground (kinematically consistent, gated)
Terrain Height = Altitude - Range

### Use cases
- Improve Altitude Hold stabilization from wind gusts, GPS errors, and vibration.
- Fixed wing landing. Terrain Height is imporant for glide slope.
- Multi-Copter takeoff height accuracy. GPS accuracy is poor while on the ground and barometer suffers from ground effect.
- Multi-Copter landing. Same as above.
- Optical Flow. Velocity estimate relies on Range to convert from Pixel_Vx/Vy to Vehicle_Vx/Vy.
- Anything else?

### Description

