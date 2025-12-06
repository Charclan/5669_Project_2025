cmake_minimum_required(VERSION 3.0.2)
project(ucf_go1_control)

set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  trajectory_msgs
  hardware_interface
  unitree_legged_sdk
  ros_control_boilerplate
)

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
  include
  third-party/unitree_guide/include
  ${catkin_INCLUDE_DIRS}
  ${unitree_legged_sdk_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(unitree_guide
  third-party/unitree_guide/src/common/LowPassFilter.cpp
  third-party/unitree_guide/src/common/unitreeLeg.cpp
  third-party/unitree_guide/src/common/unitreeRobot.cpp
  third-party/unitree_guide/src/control/Estimator.cpp
)

add_library(${PROJECT_NAME}
 src/body_controller.cpp
)

# Go1 Hardware Interface
add_library(go1_hw_interface
  src/go1_hw_interface.cpp
)
target_link_libraries(go1_hw_interface
  ${EXTRA_LIBS}
  ${catkin_LIBRARIES}
)
# add_dependencies(go1_hw_interface
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_LIBRARIES}
#   ${catkin_EXPORTED_TARGETS})

# Main control executable
add_executable(go1_hw_main src/go1_hw_main.cpp)
target_link_libraries(go1_hw_main
  go1_hw_interface
  ${EXTRA_LIBS}
  ${catkin_LIBRARIES}
  # ${catkin_EXPORTED_TARGETS}
)
# add_dependencies(go1_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/ucf_go1_control_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  unitree_guide
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node go1_hw_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install libraries
install(TARGETS
    go1_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY third-party/unitree_guide/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "**.h"
  PATTERN ".svn" EXCLUDE
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY profiles/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/profiles
)