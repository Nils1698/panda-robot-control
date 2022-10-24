cmake_minimum_required(VERSION 3.0.2)
project(nils_master_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
apriltag_ros
cv_bridge
geometry_msgs
image_transport
message_generation
roscpp
rospy
sensor_msgs
std_msgs
tf2
tf2_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PredictedPoses.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
 CATKIN_DEPENDS apriltag_ros cv_bridge geometry_msgs image_transport message_runtime roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#################
## Exceutables ##
#################

add_executable(pubRobotPose src/pubRobotPose.cpp)
target_link_libraries(pubRobotPose ${catkin_LIBRARIES})
target_include_directories(pubRobotPose SYSTEM PUBLIC
)

add_executable(prediction_handler src/prediction_handler.cpp)
target_link_libraries(prediction_handler ${catkin_LIBRARIES})
target_include_directories(prediction_handler SYSTEM PUBLIC
)

add_executable(publish_apriltag_pose src/publish_apriltag_pose.cpp)
target_link_libraries(publish_apriltag_pose ${catkin_LIBRARIES})
target_include_directories(publish_apriltag_pose SYSTEM PUBLIC
)

add_executable(publish_tf_apriltag_pose_to_predictor src/publish_tf_apriltag_pose_to_predictor.cpp)
target_link_libraries(publish_tf_apriltag_pose_to_predictor ${catkin_LIBRARIES})
target_include_directories(publish_apriltag_pose SYSTEM PUBLIC
)

add_executable(publish_tf_apriltag_pose_to_controller src/publish_tf_apriltag_pose_to_controller.cpp)
target_link_libraries(publish_tf_apriltag_pose_to_controller ${catkin_LIBRARIES})
target_include_directories(publish_apriltag_pose SYSTEM PUBLIC
)

add_executable(publish_tf_apriltag_pose_to_controller_gripper src/publish_tf_apriltag_pose_to_controller_gripper.cpp)
target_link_libraries(publish_tf_apriltag_pose_to_controller_gripper ${catkin_LIBRARIES})
target_include_directories(publish_tf_apriltag_pose_to_controller_gripper SYSTEM PUBLIC
)

add_executable(publish_tf_apriltag_pose_to_predictor_gripper src/publish_tf_apriltag_pose_to_predictor_gripper.cpp)
target_link_libraries(publish_tf_apriltag_pose_to_predictor_gripper ${catkin_LIBRARIES})
target_include_directories(publish_tf_apriltag_pose_to_predictor_gripper SYSTEM PUBLIC
)

#Files to pick up a static file at an known location

add_executable(publish_hardcoded_coordinates src/pick-up-static-known/publish_hardcoded_coordinates.cpp)
target_link_libraries(publish_hardcoded_coordinates ${catkin_LIBRARIES})
target_include_directories(publish_hardcoded_coordinates SYSTEM PUBLIC
)

#############
## Testing ##
#############

