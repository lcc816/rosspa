cmake_minimum_required(VERSION 2.8.3)
project(example_component)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS spa_core roslib)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subpose
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES})

install(TARGETS example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the xteds file
install(FILES xteds/Thermometer_Demo.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/xteds
)
