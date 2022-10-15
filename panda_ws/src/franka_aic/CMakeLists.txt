cmake_minimum_required(VERSION 2.8.3)
project(franka_aic)

set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11) # Er det fordi jeg compiler med c++11 frem for c++14?

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
  # addede due to HIRO
  kdl_parser
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  trac_ik_lib
  # added updated versions of packages from HIRO
  tf
  # tf2
  # tf2_ros
)

find_package(Eigen3 REQUIRED)
# find_package(Franka 0.3.0 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

add_message_files(FILES
  JointTorqueComparison.msg
)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_aic
  CATKIN_DEPENDS
    controller_interface
    franka_control
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
  DEPENDS Franka
)

add_library(franka_aic
  src/panda_utils/panda_trac_ik.cpp
  src/AIC_controller.cpp
  src/MRAC_controller.cpp
  src/AICpanda.cpp
##  src/ckk_AIC_controller.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_dependencies(franka_aic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_aic PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_aic SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_aic PUBLIC
  include
)

## Installation
install(TARGETS franka_aic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_aic_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_aic FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_aic
    FILES ${SOURCES}
    DEPENDS franka_aic
  )
endif()
