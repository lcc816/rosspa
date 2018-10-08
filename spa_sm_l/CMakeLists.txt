cmake_minimum_required(VERSION 2.8.3)
project(spa_sm_l)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_srvs
    message_generation
    spa_core
)

## Generate services in the 'srv' folder
add_service_files(FILES Hello.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_srvs)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS roscpp std_srvs message_runtime)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(spa_sm_l src/spa_sm_l.cpp)
add_dependencies(spa_sm_l ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spa_sm_l  ${catkin_LIBRARIES})
