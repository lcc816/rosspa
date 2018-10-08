cmake_minimum_required(VERSION 2.8.3)
project(spa_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    std_msgs
    message_generation
    actionlib_msgs
)

add_action_files(
    DIRECTORY action
    FILES SpaProbe.action
)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES spa_core
    CATKIN_DEPENDS std_msgs actionlib actionlib_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(spa_core src/spa_application.cpp)
add_dependencies(spa_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spa_core ${catkin_LIBRARIES})
