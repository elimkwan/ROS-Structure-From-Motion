cmake_minimum_required(VERSION 3.0.2)
project(duck_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  CATKIN_DEPENDS rospy std_msgs argparse numpy laser_geometry
#  INCLUDE_DIRS include
#  LIBRARIES duck_demo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/action_hector.py
  scripts/action_openloop.py
  scripts/action_rrt.py
  scripts/action_astar.py
  scripts/action_frontier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

