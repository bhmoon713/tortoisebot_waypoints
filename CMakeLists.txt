cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_waypoints)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf
  message_generation
  rostest
)

add_action_files(
  DIRECTORY action
  FILES WaypointAction.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs std_msgs nav_msgs tf
)

catkin_install_python(PROGRAMS
  scripts/tortoisebot_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_rostest(tests/waypoints_test.test)
endif()
