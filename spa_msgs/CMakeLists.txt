cmake_minimum_required(VERSION 2.8.3)
project(spa_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    std_msgs 
    geometry_msgs
    actionlib_msgs
    actionlib
)

add_message_files(
    DIRECTORY msg
    FILES
    SpaRequestLsProbe.msg
    GetTemperature.msg
)

add_action_files(
    DIRECTORY action
    FILES
    SpaQuery.action
)

add_service_files(
    DIRECTORY srv
    FILES
    Hello.srv
    SpaProbe.srv
    SpaXteds.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs std_msgs actionlib_msgs actionlib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
