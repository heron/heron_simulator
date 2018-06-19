# Heron USV Simulation

## Installation

In your catkin workspace, run the command:  
```git clone https://github.com/uuvsimulator/uuv_simulator```    
```git clone https://github.com/heron/heron_simulator```

Then build your workspace and you're ready to run the simulation!

## Running the Simulation

To simulate the Heron and the world, run the following command:  
```roslaunch heron_gazebo heron_world.launch```

The simulation can be set to change what world it launches. If the launch file is contained within the launch folder of a package (as it usually is), the following commmand must be run:  
```roslaunch heron_gazebo heron_world.launch world_pkg:=[PACKAGE] world_pkg_file:=[WORLD_FILE]```  

If an absolute file path must be provided:  
```roslaunch heron_gazebo heron_world.launch use_pkg_path:=0 world_file:=[FILE PATH]```  

Optionally, multiple Heron USVs can be simulated at the same time, as long as each robot is under a unique namespace.

To do this, run the following commands in separate terminals:  
```roslaunch [PACKAGE_CONTAINING_WORLD] [WORLD_LAUNCH_FILE]```  
```roslaunch heron_gazebo heron_sim.launch```  
```roslaunch heron_gazebo heron_sim.launch namespace:=heron1```  
```roslaunch heron_gazebo heron_sim.launch namespace:=heron2```  
..., etc.

By default, the namespace is *heron*. Of course, you can name the namespaces anything you want.

Using ```PACKAGE_CONTAINING_WORLD=uuv_descriptions```, there are pre-made worlds for ```WORLD_FILE```:
  - ocean_waves.launch
  - mangalia.launch
  - munkholmen.launch
  - empty_underwater_world.launch
  - herkules_ship_wreck.launch
  - lake.launch

**Note: ** "empty_underwater_world" and "herkules_ship_wreck" have very dark oceans, making them difficult to use.

To create your own world file, follow these instructions:  https://uuvsimulator.github.io/tutorials/seabed_world.html

## Control

The Heron is controlled used interactive markers in RViz. One control drives the Heron forward/backward. The other control causes rotation. There are quite a number of transform frames from which to control the Heron.

Most recommended is the Heron's base frame: *[namespace]/base_link*. However, the Heron's world location won't be shown. If this location must be seen, the frame *[namespace]/odom* can be used but will have the Heron's location constantly fluctuating (due to the GPS updates) which can be hard to use.

When simulating multiple Herons, their transform frames are connected via the *utm* frame. Technically, this frame could be used to visualize the Herons but, due to its globalness, it can be difficult to find the Herons in the visualization. Regardless, when controlling a Heron from its *[namespace]/base_link* frame, you can visualize and control the other Herons.

## Topics

All topics and transform frames were put under the *heron* namespace to allow for multiple robots in a single simulation without interference.

The simulated Heron uses the same control topics as the actual Heron. Simulation uses the heron_controller package to control itself.
  - Simply publish on the */heron/cmd_course*, */heron/cmd_wrench*, */heron/cmd_helm* to use it
  - To publish directly to the thrusters, publish on */heron/cmd_drive*

Due to using the *heron* namespace, a change was made to the *heron_bringup* package:
  - Only difference can be found in the calibrate_compass script, to use the correct topics

## Sensors

The Heron has two primary sensors: GPS and IMU. The simulated IMU represents the ADIS16448 IMU. The magnetometer is simulated separately from the IMU (although it publishes under the "imu" prefix) and represents the Bosch BMX055 sensor.

GPS Sensor output is multiplied by "-1" so as to make Latitude increase as the Heron swims North and Longitude increase as the Heron swims East.

When calibrating the magnetometer (using the *calibrate_compass* script in *heron_bringup* package), the environment variable ROBOT_NAMESPACE must be set to the robot's namespace.

Since the EKF Sensor processing node does not expect the robot to teleport (i.e. have its pose drastically change abruptly), the Heron's odometry will lag behind if you move the Heron using Gazebo's move/rotate tools.

### Sensor Topics:

**Note: ** The following topics should all be prefaced by "/[namespace]/" where [namespace] is the robot's namespace.

| Name | Msg | Publisher | Desc |
| ---- | --- | --------- | ---- |
|imu/data_raw| Imu | Gazebo | Raw simulated IMU data|
|imu/data| Imu | imu_filter_madgwick | Filtered IMU data|
|imu/rpy| Vector3Stamped| rpy_translator.py | Raw Simulated Roll/Pitch/Yaw of IMU |
|imu/rpy/filtered | Vector3Stamped | imu_filter_madgwick | Filtered Roll/Pitch/Yaw of IMU|
|imu/mag_sim|MagneticField|Gazebo|Raw Simulated Magnetometer data|
|imu/mag_raw|Vector3Stamped|mag_interpreter.py|Raw Simulated Magnetometer data|
|imu/mag|MagneticField|mag_interpreter.py|Calibrated Magnetometer data|
|navsat/fix|NavSatFix|Gazebo|Raw Simulated GPS data|

## Hydrodynamics

To view the hydrodynamic forces acting on the Heron, add "hydro_debug:=1" as a parameter when launching *heron_sim.launch* or *heron_world.launch*. This will cause a few debug topics to be published with the "/debug/" prefix.

### Buoyancy Forces
The Heron's buoyancy forces are modelled using the Linear (Small Angle) Theory for Box-Shaped Vessels, described in section 4.2 of: http://www.fossen.biz/wiley/Ch4.pdf

The metacentric heights were tuned rather than calculated. The Heron was measured/estimated to sit 0.06 metres into the water (i.e. the submerged_height parameter) without any extra weight.

With the antennae, the Heron is technically 0.74 metres tall, but these antennae are neglibly small and this extra height would make the simulation much less precise when the Heron is submerged. So the Heron's height has been set to 0.32 metres (i.e. the height ignoring the antennae).

### Damping Forces

The damping forces are modelled as quasi-quadratic. For each axis, where "Q" is the quadratic damping coefficient, "L" is the linear damping coefficient, "v" is the velocity along that axis:  
``` force = -Qv|v| - Lv ```

Damping force coefficients were manually tuned to produce an accurate simulation, not calculated based off of real-world measurements.

The Gazebo tool to apply force/torque can be used, however a large enough force/torque may cause the simulation to crash. This is due to the damping force and Heron's velocity getting caught in an "amplifying loop". That is, the Heron's velocity results in a strong damping force that causes an even larger velocity which results in a stronger damping force, etc. To fix this, reduce the damping force coefficients of the offending axis to near-zero. Then slowly increase the coefficients until the simulation is acting appropriately. This problem should be fixed, but has a tendency to return for unknown reasons.

### Thrust Forces

The thrust force was calculated by measuring the acceleration of the Heron in trial runs at Columbia Lake. *uuv_simulator* calculates the thrust force of the propellors using linear interpolation techniques. The mapping is below and roughly matches the specified max and min thrust described in the Heron's datasheet.

| Input to Propellor | Output Thrust (N) |
| ------------------ | ------------- |
|-1.0| -19.88|
| -0.8| -16.52|
| -0.6| -12.6|
| -0.4| -5.6|
| -0.2| -1.4|
| 0.0| 0.0|
| 0.2| 2.24|
| 0.4| 9.52|
| 0.6| 21.28|
| 0.8| 28.0|
| 1.0| 33.6|


Due to the nature of the simulator, the Heron will "remember" the last thrust command.
That is, even if *thrusters/0/input* is no longer sending a specific thrust command, the thrusters will keep running with that thrust command. To stop the Heron, a "zero message" must be sent, whether through *cmd_drive*, *cmd_wrench*, *thrusters/0/input*, etc. The control interface using the RViz interactive markers has intermediary code to solve this issue.


## Launch Files

The *heron_world.launch* file simply launches a world file and the *heron_sim.launch*. Due to the possibility of having multiple robots, all Heron nodes are in *heron_sim.launch*.

Nodes in *heron_gazebo/heron_sim.launch*:
  - Launches *imu_filter_madgwick* to process the IMU and magnetometer data to get a more accurate Imu message.

  - Launches *heron_controller* to convert *cmd_wrench*, etc to *cmd_drive*

  - Launches *control.launch* which contains the robot's localization nodes (i.e. robot_localization)

  - Launches *description.launch* to publish the robot state

  - Runs *urdf_spawner* to spawn the robot in Gazebo

  - The simulation expects thruster inputs as two different topics (*thrusters/0/input* and *thrusters/1/input* for right and left respectively). So *heron_sim.launch* also launches a script (*cmd_drive_translate.py*) that translates cmd_drive topic from heron_controller to these topics.

  - The simulation publishes uncalibrated magnetometer values in NED frame. The IMU sensor driver would take care of this on the actual Heron, so there's a script (*mag_interpreter.py*) in *heron_sim.launch* calibrates the magnetometer values and transforms them to ENU.

  - The IMU usually publishes a *imu/rpy* topic as well (important for magnetometer calibration) so a script (*rpy_translator.py*) in description.launch translates the quaternion in *imu/data* to the *imu/rpy* topic

  - The *interactive_marker_twist_server* node publishes a geometry_msgs/Twist message but *heron_controller* expects a geometry_msgs/Wrench message. A script (*twist_translate.py*) scales and republishes the output of *interactive_marker_twist_server* with the Wrench message.

## Changes made to UUV Simulator

One change has already been mentioned under the "Sensors" topic: the simulated GPS has its output multiplied by "-1".

The other change was rewriting the calculation of how deep the Heron was currently submerged. This is found at Line 139 of HydrodynamicModel.cc (uuv_simulator/uuv_gazebo_plugins/uuuv_gazebo_plugins/src/HydrodynamicModel.cc). The old code is commented out directly below.

At line 203 of *uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_plugins/src/BuoyantObject.cc*, the buoyancy force was transformed to be relative to robot's frame of reference. This is contrary to the Gazebo documentation for *AddForceAtRelativePosition()* (which is what the force is sent to) but the simulation seems to work more accurately with this rotation being applied.

## Known Issues:

In general, the simulation doesn't do well with the vessels outside of the water. For example, the water damping forces are the identical even if the vessel is not touching the water. Since the Heron can't fly, this was assumed to be an unimportant issue.

This problem should be fixe21.28d but has a tendency to return: The Gazebo tool to apply force/torque can be used, however a large enough force/torque may cause the simulation to crash. This is due to the damping force and Heron's velocity getting caught in an "amplifying loop". That is, the Heron's velocity results in a strong damping force that causes an even larger velocity which results in a stronger damping force, etc. To fix this, reduce the damping force coefficients of the offending axis to near-zero. Then slowly increase the coefficients until the simulation is acting appropriately.
