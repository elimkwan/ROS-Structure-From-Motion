cmake_minimum_required(VERSION 3.0.2)
project(duck_action)

find_package(catkin REQUIRED)

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   duck_action_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
