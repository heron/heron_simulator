cmake_minimum_required(VERSION 2.8.3)
project(heron_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   scripts/activate_control_service
   scripts/cmd_drive_translate
   scripts/navsat_vel_translate
   scripts/rpy_translator
   scripts/twist_translate
   scripts/vector3_to_magnetic_field
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
