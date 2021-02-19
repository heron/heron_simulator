cmake_minimum_required(VERSION 3.0.2)
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

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
