# Collision Prevention

_Collision Prevention_ may be used to automatically slow and stop a vehicle before it can crash into an obstacle.
It can be enabled for multicopter vehicles when using acceleration-based [Position mode](../flight_modes_mc/position.md) (or VTOL vehicles in MC mode).

It can be enabled for multicopter vehicles in [Position mode](../flight_modes_mc/position.md) (with [MPC_POS_MODE](#MPC_POS_MODE) set to `Acceleration based`), and can use sensor data from an offboard companion computer, offboard rangefinders over MAVLink, a rangefinder attached to the flight controller, or any combination of the above.

Collision prevention limits the vehicle's speed to what it can stop from within the range its sensors see.
In directions no sensor has seen, the pilot may still fly, but only as fast as if an obstacle stood just beyond [CP_DIST](#CP_DIST).

:::tip
If high flight speeds are critical, consider disabling collision prevention when not needed.
:::

## Overview

The vehicle keeps [CP_DIST](#CP_DIST) from every obstacle its sensors have seen.
Collision Prevention never steers or yaws: a command straight into an obstacle stops short of it, and a command past it slides along it, but only into space the sensors have seen.
An obstacle already inside `CP_DIST` is backed away from.

Users are notified through _QGroundControl_ while _Collision Prevention_ is actively controlling velocity setpoints.

PX4 software setup is covered in the next section.
If you are using a distance sensor attached to your flight controller for collision prevention, it will need to be attached and configured as described in [PX4 Distance Sensor](#rangefinder).
If you are using a companion computer to provide obstacle information see [companion setup](#companion) below.

## Supported Rangefinders {#rangefinder}

### Lanbao PSK-CM8JL65-CC5 [Discontinued]

At time of writing PX4 allows you to use the [Lanbao PSK-CM8JL65-CC5](../sensor/cm8jl65_ir_distance_sensor.md) IR distance sensor for collision prevention “out of the box”, with minimal additional configuration:

- First [attach and configure the sensor](../sensor/cm8jl65_ir_distance_sensor.md), and enable collision prevention (as described above, using [CP_DIST](#CP_DIST)).
- Set the sensor orientation using [SENS_CM8JL65_R_0](../advanced_config/parameter_reference.md#SENS_CM8JL65_R_0).

### LightWare LiDAR SF45 Rotating Lidar

PX4 v1.14 (and later) supports the [LightWare LiDAR SF45](../sensor/sf45_rotating_lidar.md) rotating lidar which provides 320 degree sensing.

### Sony AS-DT1 LiDAR

PX4 supports the [Sony AS-DT1](../sensor/sony_asdt1.md) multipoint LiDAR as a directly connected UART sensor for collision prevention.
The driver publishes measurements to `obstacle_distance` with 5 degree bins, using the configured sensor yaw offset.

The AS-DT1 covers a forward horizontal field of view of about 35 degrees.
Only the covered sectors are populated; other directions remain no-data unless covered by another sensor.
Configure the sensor as described in the [Sony AS-DT1](../sensor/sony_asdt1.md) guide, then enable collision prevention with [CP_DIST](#CP_DIST).

### Other Rangefinders

Other sensors may be enabled, but this requires modification of driver code to set the sensor orientation and field of view.

- Attach and configure the distance sensor on a particular port (see [sensor-specific docs](../sensor/rangefinders.md)) and enable collision prevention using [CP_DIST](#CP_DIST).
- Modify the driver to set the orientation.
  This should be done by mimicking the `SENS_CM8JL65_R_0` parameter (though you might also hard-code the orientation in the sensor _module.yaml_ file to something like `sf0x start -d ${SERIAL_DEV} -R 25` - where 25 is equivalent to `ROTATION_DOWNWARD_FACING`).
- Modify the driver to set the _field of view_ in the distance sensor UORB topic (`distance_sensor_s.h_fov`).

## PX4 (Software) Setup

Configure collision prevention by [setting the following parameters](../advanced_config/parameters.md) in _QGroundControl_:

| Parameter                                                                                       | Description                                                                                                                                                                                                                                                                                     |
| ----------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="CP_DIST"></a>[CP_DIST](../advanced_config/parameter_reference.md#CP_DIST)                | Set the minimum allowed distance (the closest distance that the vehicle can approach the obstacle). Set negative to disable _collision prevention_. <br>> **Warning** This value is the distance to the sensors, not the outside of your vehicle or propellers. Be sure to leave a safe margin! |
| <a id="CP_DIST_V"></a>[CP_DIST_V](../advanced_config/parameter_reference.md#CP_DIST_V)          | Set the gap to keep between the top or bottom of the vehicle and an obstacle over or under it. Only used with the obstacle map. See [Above and Below](#above_below).                                                                                                                            |
| <a id="CP_DELAY"></a>[CP_DELAY](../advanced_config/parameter_reference.md#CP_DELAY)             | Set the sensor and velocity setpoint tracking delay. See [Delay Tuning](#delay_tuning) below.                                                                                                                                                                                                   |
| <a id="MPC_POS_MODE"></a>[MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE) | Must be set to `Acceleration based`.                                                                                                                                                                                                                                                            |

## Algorithm Description

The data from all sensors are fused into 72 sectors of 5 degrees around the vehicle, each holding the nearest obstacle, the sensor's range if nothing is in range, or no data.

Each obstacle limits the speed towards its nearest point, the sector closer than its neighbours, to what the vehicle can stop from before [CP_DIST](#CP_DIST).
Flying along a wall keeps the distance to it, so the wall does not slow it; flying at it does.
Along the velocity and the stick direction the speed is also limited to what the vehicle can stop from within the range the sensors have seen clear, or, where they have not looked, within `CP_DIST`.
The commanded acceleration is changed as little as these limits allow.
If that turns it from space the sensors have seen towards space they have not, the vehicle stops instead of sliding there.
This takes into account [MPC_JERK_MAX](../advanced_config/parameter_reference.md#MPC_JERK_MAX), [MPC_ACC_HOR](../advanced_config/parameter_reference.md#MPC_ACC_HOR) and [MPC_XY_VEL_P_ACC](../advanced_config/parameter_reference.md#MPC_XY_VEL_P_ACC).

Delay, both in the vehicle tracking velocity setpoints and in receiving sensor data from external sources, is conservatively estimated via the [CP_DELAY](#CP_DELAY) parameter.
This should be [tuned](#delay_tuning) to the specific vehicle.

### Above and Below {#above_below}

With the `obstacle_map` module, Collision Prevention also keeps [CP_DIST_V](#CP_DIST_V) between the top or bottom of the vehicle and what is over or under it.
The map moves the vehicle's body, a cylinder of [OMAP_VEH_RAD](../advanced_config/parameter_reference.md#OMAP_VEH_RAD) and [OMAP_VEH_HGT](../advanced_config/parameter_reference.md#OMAP_VEH_HGT), up, down and along its motion, and finds where it would touch an obstacle.
Climbing and descending slow down to stop short of it, and an obstacle in the path brakes the motion towards it.
Descending never slows below [MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED), so the vehicle can land.
Space above or below that no sensor has seen can be climbed or descended into, at a speed the vehicle can stop from within `CP_DIST_V`.

An obstacle less than `CP_DIST_V` above or below the body counts horizontally, so the vehicle climbs until a fence is that far below it before flying over.
The ground or a ceiling directly over or under the vehicle does not count horizontally.

A forward-facing sensor sees what is over or under the vehicle only while it is still ahead: not a ceiling the vehicle flies under while pitched forward, and nothing over where it took off.

### Range Data Loss

If the autopilot does not receive range data from any sensor for longer than 0.5s, it will output a warning _No range data received, no movement allowed_.
This will force the velocity setpoints in xy to zero.
After 5 seconds of not receiving any data, the vehicle will switch into [HOLD mode](../flight_modes_mc/hold.md).
If you want the vehicle to be able to move again, you will need to disable Collision Prevention by either setting the parameter [CP_DIST](#CP_DIST) to a negative value, or switching to a mode other than [Position mode](../flight_modes_mc/position.md) (e.g. to _Altitude mode_ or _Stabilized mode_).

If you have multiple sensors connected and you lose connection to one of them, the data of the faulty sensor expires and its region is treated as unseen: you can still fly there, at the reduced speed.

### CP_DELAY Delay Tuning {#delay_tuning}

There are two main sources of delay which should be accounted for: _sensor delay_, and vehicle _velocity setpoint tracking delay_.
Both sources of delay are tuned using the [CP_DELAY](#CP_DELAY) parameter.

The _sensor delay_ for distance sensors connected directly to the flight controller can be assumed to be 0.
For external vision-based systems the sensor delay may be as high as 0.2s.

Vehicle _velocity setpoint tracking delay_ can be measured by flying at full speed in [Position mode](../flight_modes_mc/position.md), then commanding a stop.
The delay between the actual velocity and the velocity setpoint can then be measured from the logs.
The tracking delay is typically between 0.1 and 0.5 seconds, depending on vehicle size and tuning.

:::tip
If vehicle speed oscillates as it approaches the obstacle (i.e. it slows down, speeds up, slows down) the delay is set too high.
:::

### Sensor Coverage

Collision Prevention only knows what its sensors have seen.
A pilot can fly towards space no sensor covers, and does whenever the vehicle moves sideways or backwards with a single forward-facing sensor: keep the nose where the vehicle goes, or add sensors to the sides.

## Companion Setup {#companion}

::: warning
The companion implementation/setup is currently untested (the original companion project was unmaintained and has been archived).
:::

If using a companion computer or external sensor, it needs to supply a stream of [OBSTACLE_DISTANCE](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE) messages, which should reflect when and where obstacle were detected.

The minimum rate at which messages _must_ be sent depends on vehicle speed - at higher rates the vehicle will have a longer time to respond to detected obstacles.
Initial testing of the system used a vehicle moving at 4 m/s with `OBSTACLE_DISTANCE` messages being emitted at 10Hz (the maximum rate supported by the vision system).
The system may work well at significantly higher speeds and lower frequency distance updates.

## Gazebo Simulation

_Collision Prevention_ can be tested using [Gazebo](../sim_gazebo_gz/index.md) with the [x500_lidar_2d](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-2d-lidar) model.
To do this, start a simulation with the x500 lidar model by running the following command:

```sh
make px4_sitl gz_x500_lidar_2d
```

Next, adjust the relevant parameters to the appropriate values and add arbitrary obstacles to your simulation world to test the collision prevention functionality.

The diagram below shows a simulation of collision prevention as viewed in Gazebo.

![RViz image of collision detection using the x500_lidar_2d model in Gazebo](../../assets/simulation/gazebo/vehicles/x500_lidar_2d_viz.png)

## Development Information/Tools

### Plotting Obstacle Distance and Minimum Distance in Real-Time with PlotJuggler

[PlotJuggler](../log/plotjuggler_log_analysis.md) can be used to monitor and visualize obstacle distances in a real-time plot, including the minimum distance to the closest obstacle.

<lite-youtube videoid="amLheoHgwc4" title="Plotting Obstacle Distance and Minimum Distance in Real-Time with PlotJuggler"/>

To use this feature you need to add a reactive Lua script to PlotJuggler, and also configure PX4 to export [`obstacle_distance_fused`](../msg_docs/ObstacleDistance.md) UORB topic data.
The Lua script works by extracting the `obstacle_distance_fused` data at each time step, converting the distance values into Cartesian coordinates, and pushing them to PlotJuggler.

The steps are:

1. Follow the instructions in [Plotting uORB Topic Data in Real Time using PlotJuggler](../debug/plotting_realtime_uorb_data.md)
2. Configure PX4 to publish obstacle distance data (so that it is available to PlotJuggler):

   Add the [`obstacle_distance_fused`](../msg_docs/ObstacleDistance.md) UORB topic to your [`dds_topics.yaml`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) so that it is published by PX4:

   ```sh
   - topic: /fmu/out/obstacle_distance_fused
     type: px4_msgs::msg::ObstacleDistance
   ```

   For more information see [DDS Topics YAML](../middleware/uxrce_dds.md#dds-topics-yaml) in [uXRCE-DDS](../middleware/uxrce_dds.md) (PX4-ROS 2/DDS Bridge)\_.

3. Open PlotJuggler and navigate to the **Tools > Reactive Script Editor** section.
   In the **Script Editor** tab, add following scripts in the appropriate sections:
   - **Global code, executed once:**

     ```lua
     obs_dist_fused_xy = ScatterXY.new("obstacle_distance_fused_xy")
     obs_dist_min = Timeseries.new("obstacle_distance_minimum")
     ```

   - **function(tracker_time)**

     ```lua
     obs_dist_fused_xy:clear()

     i = 0
     angle_offset = TimeseriesView.find("/fmu/out/obstacle_distance_fused/angle_offset")
     increment = TimeseriesView.find("/fmu/out/obstacle_distance_fused/increment")
     min_dist = 65535

     -- Cache increment and angle_offset values at tracker_time to avoid repeated calls
     local angle_offset_value = angle_offset:atTime(tracker_time)
     local increment_value = increment:atTime(tracker_time)

     if increment_value == nil or increment_value <= 0 then
         print("Invalid increment value: " .. tostring(increment_value))
         return
     end

     local max_steps = math.floor(360 / increment_value)

     while i < max_steps do
         local str = string.format("/fmu/out/obstacle_distance_fused/distances[%d]", i)
         local distance = TimeseriesView.find(str)
         if distance == nil then
             print("No distance data for: " .. str)
             break
         end

         local dist = distance:atTime(tracker_time)
         if dist ~= nil and dist < 65535 then
             -- Calculate angle and Cartesian coordinates
             local angle = angle_offset_value + i * increment_value
             local y = dist * math.cos(math.rad(angle))
             local x = dist * math.sin(math.rad(angle))

             obs_dist_fused_xy:push_back(x, y)

             -- Update minimum distance
             if dist < min_dist then
                 min_dist = dist
             end
         end

         i = i + 1
     end

     -- Push minimum distance once after the loop
     if min_dist < 65535 then
         obs_dist_min:push_back(tracker_time, min_dist)
     else
         print("No valid minimum distance found")
     end
     ```

4. Enter a name for the script on the top right, and press **Save**.
   Once saved, the script should appear in the _Active Scripts_ section.
5. Start streaming the data using the approach described in [Plotting uORB Topic Data in Real Time using PlotJuggler](../debug/plotting_realtime_uorb_data.md).
   You should see the `obstacle_distance_fused_xy` and `obstacle_distance_minimum` timeseries on the left.

Note that you have to press **Save** again to re-enable the scripts after loading a new log file or otherwise clearing data.

### Sensor Data Overview

Collision Prevention has an internal obstacle distance map that divides the plane around the drone into 72 Sectors.
Internally this information is stored in the [`obstacle_distance`](../msg_docs/ObstacleDistance.md) UORB topic.
New sensor data is compared to the existing map, and used to update any sections that has changed.

The angles in the `obstacle_distance` topic are defined as follows:

![Obstacle_Distance Angles](../../assets/computer_vision/collision_prevention/obstacle_distance_def.svg)

The data from rangefinders, rotary lidars, or companion computers, is processed differently, as described below.

#### Rotary Lidars

Rotary Lidars add their data directly to the [`obstacle_distance`](../msg_docs/ObstacleDistance.md) uORB topic.

#### Rangefinders

Rangefinders publish their data to the [`distance_sensor`](../msg_docs/DistanceSensor.md) uORB topic.

This data is then mapped onto the `obstacle_distance` topic.
All sectors which have any overlap with the orientation (`orientation` and `q`) of the rangefinder, and the horizontal field of view (`h_fov`) are assigned that measurement value.
For example, a distance sensor measuring from 9.99° to 10.01° the measurements will get added to the bin's corresponding to 5° and 10° covering the arc from 2.5° and 12.5°

::: info
the quaternion `q` is only used if the `orientation` is set to `ROTATION_CUSTOM`.
:::

#### Companion Computers

Companion computers update the `obstacle_distance` topic using ROS 2 or the [OBSTACLE_DISTANCE](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE) MAVLink message.

<!-- to edit the image, open it in inkscape -->
