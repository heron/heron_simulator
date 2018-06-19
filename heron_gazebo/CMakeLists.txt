cmake_minimum_required(VERSION 2.8.3)
project(heron_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   scripts/cmd_drive_translate.py
 	 scripts/mag_interpreter.py
 	 scripts/rpy_translator.py
	 scripts/twist_translate.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
