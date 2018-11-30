cmake_minimum_required(VERSION 2.8.3)
project(spa_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    spa_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES spa_core
    CATKIN_DEPENDS roscpp spa_msgs
    DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(spa_core src/spa_application.cpp src/xteds_repository.cpp src/uuid.cpp)
add_dependencies(spa_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spa_core ${catkin_LIBRARIES} crypto)

install(TARGETS spa_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install sub folders spa_core/ and rapidxml/  to the global include directory
install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
