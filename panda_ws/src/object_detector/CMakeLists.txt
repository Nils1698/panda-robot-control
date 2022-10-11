cmake_minimum_required(VERSION 3.0.2)
project(object_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
 # message_runtime
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  geometry_msgs
  paic
  tf2
  tf2_ros
  apriltag_ros
  # message_generation
)
find_package(OpenCV 4.2.0 EXACT REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Num.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs  std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detector
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport tf2 # message_runtime  
#  CATKIN_DEPENDS message_generation
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  paic
)

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_dependencies(image_converter qr_code_detector_wrapper/src/qr_code_detector_wrapper.cpp)

add_executable(object_detector src/object_detector.cpp)
target_link_libraries(object_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(qr_basic src/qr_basic.cpp)
target_link_libraries(qr_basic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(target_selector src/target_selector.cpp)
target_link_libraries(target_selector ${catkin_LIBRARIES})

add_executable(pose_tf_cam_to_panda src/pose_tf_cam_to_panda.cpp)
target_link_libraries(pose_tf_cam_to_panda
 ${catkin_LIBRARIES}
)


#add_executable(qr_detector src/qr_detector.cpp)
#target_link_libraries(qr_detector ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
# add_dependencies(qr_detector qr_detector)

# target_link_libraries(DisplayImage ${OpenCV_LIBS})

