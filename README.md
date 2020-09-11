# Heron USV Simulation

## Installation

Use the instructions for ROS Kinetic and Gazebo 7 at the following link:   
https://uuvsimulator.github.io/installation/


Then in the src folder of your catkin workspace, run the commands:  
  ```
git clone https://github.com/heron/heron_simulator  
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -yr
  ```

Also ensure you have the following packages in your src folder: heron, heron_controller, heron_desktop, and imu_tools. Build your workspace and you're ready to run the simulation!

## Running the Simulation

To simulate the Heron and the world, run the following command:
```roslaunch heron_gazebo heron_world.launch```

The simulation can be set to change what world it launches. If the launch file is contained within the launch folder of a package (as it usually is), the following commmand must be run:
```roslaunch heron_gazebo heron_world.launch world_pkg:=[PACKAGE] world_pkg_file:=[WORLD_FILE]```  

If an absolute file path must be provided:  
```roslaunch heron_gazebo heron_world.launch use_pkg_path:=0 world_file:=[FILE PATH]```  

Optionally, multiple Heron USVs can be simulated at the same time, as long as each robot is under a unique namespace.

To do this, run the following commands in separate terminals:  
```
roslaunch [PACKAGE_CONTAINING_WORLD] [WORLD_LAUNCH_FILE]
roslaunch heron_gazebo heron_sim.launch
roslaunch heron_gazebo heron_sim.launch namespace:=heron1  
roslaunch heron_gazebo heron_sim.launch namespace:=heron2  
..., etc.
```

By default, the namespace is empty. Of course, you can name the namespaces anything you want.

Using ```PACKAGE_CONTAINING_WORLD=uuv_descriptions```, there are pre-made worlds for ```WORLD_FILE```:
  - ocean_waves.launch
  - mangalia.launch
  - munkholmen.launch
  - empty_underwater_world.launch
  - herkules_ship_wreck.launch
  - lake.launch

**Note: ** "empty_underwater_world" and "herkules_ship_wreck" have very dark oceans, making them difficult to use.

Starting the Heron in any configuration that results in immediate acceleration upon spawn will interfere with the IMU initialization (see Known Issues), so only the x, y, and yaw of the starting location of the Heron can be changed:

```roslaunch heron_gazebo heron_[sim/world].launch x:=[X_METRES] y:=[Y_METRES] yaw:=[YAW_RADIANS]```

## Control

The Heron is controlled used interactive markers in RViz. One control drives the Heron forward/backward. The other control causes rotation. There are quite a number of transform frames from which to control the Heron.

Most recommended is the Heron's base frame: *[namespace]/base_link*. However, the Heron's world location won't be shown. If this location must be seen, the frame *[namespace]/odom* can be used but will have the Heron's location constantly fluctuating (due to the GPS updates) which can be hard to use.

When simulating multiple Herons, their transform frames are connected via the *utm* frame. Technically, this frame could be used to visualize the Herons but, due to its globalness, it can be difficult to find the Herons in the visualization.

In any transform frame, all the Herons can be controlled via RViz. You will have to add RobotModel and InteractiveMarker to RViz for each Heron.

## Topics

When running the simulation, a custom namespace can be set to prefix all the transform frames and topic names. An empty namespace can be used but, when doing so, the thruster topics used by *uuv_simulator* will be placed under the *heron* namespace. This means that an empty namespace and a *heron* namespace cannot be used at the same time.

The simulated Heron uses the same control topics as the actual Heron. Simulation uses the heron_controller package to control itself. Prefix the topic names below with the simulation's custom namespace.
  - Simply publish on the *cmd_course*, *cmd_wrench*, *cmd_helm* to use it
  - To publish directly to the thrusters, publish on *cmd_drive*

## Sensors

The Heron has two primary sensors: GPS and IMU. The simulation uses the corresponding hector_gazebo plugins as well as the magnetometer plugin.

When calibrating the magnetometer (using the *calibrate_compass* script in *heron_bringup* package), the environment variable ROBOT_NAMESPACE must be set to the robot's namespace.

Since the EKF Sensor processing node does not expect the robot to teleport (i.e. have its pose drastically change suddenly), the Heron's odometry will lag behind if you move the Heron using Gazebo's move/rotate tools.

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
|navsat/velocity|Vector3Stamped|Gazebo|Simulated Velocity Data in NWU|
|navsat/vel|TwistStamped|navsat_vel_translate.py|Simulated Velocity Data in ENU|
|navsat/vel_cov|TwistWithCovarianceStamped|vel_cov.py|ENU Velocity Data with *approximate* Covariance|

## Hydrodynamics

To view the hydrodynamic forces acting on the Heron, add "hydro_debug:=1" as a parameter when launching *heron_sim.launch* or *heron_world.launch*. This will cause a few debug topics to be published with the "/debug/" prefix.

### Buoyancy Forces
The Heron's buoyancy forces are modelled using the Linear (Small Angle) Theory for Box-Shaped Vessels, described in section 4.2 of: http://www.fossen.biz/wiley/Ch4.pdf

The metacentric heights were tuned rather than calculated. The Heron was measured/estimated to sit 0.02 metres into the water (i.e. the submerged_height parameter) without any extra weight.

With the antennae, the Heron is technically 0.74 metres tall, but these antennae are neglibly small and this extra height would make the simulation much less precise when the Heron is submerged. So the Heron's height has been set to 0.32 metres (i.e. the height ignoring the antennae).

### Damping Forces

The damping forces are modelled as quasi-quadratic. For each axis, where "Q" is the quadratic damping coefficient, "L" is the linear damping coefficient, "v" is the velocity along that axis:  
``` force = -Qv|v| - Lv ```

Damping force coefficients were manually tuned to produce an accurate simulation, not calculated based off of real-world measurements.

The Gazebo tool to apply force/torque can be used, however a large enough force/torque may cause the simulation to crash. This is due to the damping force and Heron's velocity getting caught in an "amplifying loop". That is, the Heron's velocity results in a strong damping force that causes an even larger velocity which results in a stronger damping force, etc. To fix this, reduce the damping force coefficients of the offending axis to near-zero. Then slowly increase the coefficients until the simulation is acting appropriately. The simulator should no longer have this issue but the problem has a confusing tendency to return.

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

  - The IMU usually publishes a *imu/rpy* topic as well (important for magnetometer calibration) so a script (*rpy_translator.py*) in description.launch translates the quaternion in *imu/data* to the *imu/rpy* topic.

  - The *interactive_marker_twist_server* node publishes a geometry_msgs/Twist message with the linear velocity ranging from -1 to 1 m/s and the angular velocity ranging from -2.2 to 2.2 rad/s. The script *twist_translate.py* scales the linear velocity according to the contents of *config/heron_controller.yaml* and the angular velocity from -1.1 to 1.1 rad/s.

  - The Gazebo GPS plugin publishes velocity information in a NorthWestUp configuration and a Vector3Stamped msg. A script (*navsat_vel_translate.py*) converts the velocity to ENU and as a TwistStamped message.

## Creating custom worlds

Custom worlds can be created using the *heron_worlds* package. All that's needed is a 3D file (currently only STL has been tested) that represents the seabed and coastline of the environment.

## Known Issues:

In general, the simulation doesn't do well with the vessels outside of the water. For example, the water damping forces are the identical even if the vessel is not touching the water. Since the Heron can't fly, this was assumed to be an unimportant issue.

This problem should be fixed but has a tendency to return: The Gazebo tool to apply force/torque can be used, however a large enough force/torque may cause the simulation to crash. This is due to the damping force and Heron's velocity getting caught in an "amplifying loop". That is, the Heron's velocity results in a strong damping force that causes an even larger velocity which results in a stronger damping force, etc. To fix this, reduce the damping force coefficients of the offending axis to near-zero. Then slowly increase the coefficients until the simulation is acting appropriately.

When the Heron's IMU filter initializes, it requires the vessel to be floating on the water, fairly stationary. Therefore, spawning a Heron in the air or inside of another Heron will ruin the initialization. This causes the IMU data to be very incorrect for some time afterwards. Eventually, the data will correct itself and it may be quick but it also may be slow.
