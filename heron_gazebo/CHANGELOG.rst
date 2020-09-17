^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2020-09-17)
------------------
* Add the UUV worlds dependency
* Rename "heron_sim" to "spawn_heron" for compatibility with the new simulation environments
* Tidy up the lake world a little, set the camera position so the initial view is a little nicer
* Add a second world, a lake with some islands (also copied from UUV Worlds so we can modify as-necessary for a surface-only vehicle)
* Add a copy of the world to make future modifications easier. Fix the initial camera position so that it puts the robot in-frame instead of starting underwater somewhere.
* Add all the scripts to the cmake file
* Remove the .py from the node scripts & update the launch file accordingly
* Use the new (not-deprecated) MagneticField messages for the imu filter, add a simple node to translate from the old Vector3Stamped to MagneticField messges
* Disable the mag data in the simulation; the imu filter is timing out waiting for data, which is disabling the heron_control node, making the robot permanently immobile.  See RPSW-474.
* fixed location of worlds in launch file and readme
* Param tuning and constant thruster control being activated in simulation
* Added missing run dependencies
* Initialized twist_translate's zero message properly and ensured no issues with callbacks in nav_vel
* Fixed namespace functionality to allow empty namespaces
* Added custom spawning to heron_world.launch
* Fixed localization when heron has nonzero initial yaw
* New config file for simulation heron_controller params
* Fixed name+email in copyright notices
* Added NWU->ENU swapping here because it was removed from heron_control
* Fixed interactive marker's vel scaling and created new PID file for simulation
* Added translator for Vec3 to Twist for navsat/vel
* Changed covariance limits because simulated sensors are better than the actual sensors
* Removed mag_interpreter.py from CMakeLists.txt
* Fixed twist_translate.py to scale and publish Twist msgs
* Added script to convert navsat/vel from NED to ENU
* Fixed RViz interactive markers control
* Removed mag_interpreter.py because Gazebo's magnetometer is now used
* Prevented z, roll, and pitch of spawn location to be changed
* Added copyright notices to launch and script files
* Initial commit
* Contributors: Chris Iverach-Brereton, Guy Stoppi, Nirzvi1, Shreya Subramaniam
