# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "predictor: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipredictor:/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(predictor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_custom_target(_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "predictor" "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" "std_msgs/MultiArrayDimension:geometry_msgs/Point:std_msgs/Header:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:std_msgs/MultiArrayLayout:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(predictor
  "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/predictor
)

### Generating Services

### Generating Module File
_generate_module_cpp(predictor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/predictor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(predictor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(predictor_generate_messages predictor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_dependencies(predictor_generate_messages_cpp _predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(predictor_gencpp)
add_dependencies(predictor_gencpp predictor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS predictor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(predictor
  "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/predictor
)

### Generating Services

### Generating Module File
_generate_module_eus(predictor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/predictor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(predictor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(predictor_generate_messages predictor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_dependencies(predictor_generate_messages_eus _predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(predictor_geneus)
add_dependencies(predictor_geneus predictor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS predictor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(predictor
  "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/predictor
)

### Generating Services

### Generating Module File
_generate_module_lisp(predictor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/predictor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(predictor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(predictor_generate_messages predictor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_dependencies(predictor_generate_messages_lisp _predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(predictor_genlisp)
add_dependencies(predictor_genlisp predictor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS predictor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(predictor
  "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/predictor
)

### Generating Services

### Generating Module File
_generate_module_nodejs(predictor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/predictor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(predictor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(predictor_generate_messages predictor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_dependencies(predictor_generate_messages_nodejs _predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(predictor_gennodejs)
add_dependencies(predictor_gennodejs predictor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS predictor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(predictor
  "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/predictor
)

### Generating Services

### Generating Module File
_generate_module_py(predictor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/predictor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(predictor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(predictor_generate_messages predictor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda-robot-control/panda_ws/src/predictor/msg/PredictedPoses.msg" NAME_WE)
add_dependencies(predictor_generate_messages_py _predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(predictor_genpy)
add_dependencies(predictor_genpy predictor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS predictor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/predictor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(predictor_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(predictor_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(predictor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/predictor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(predictor_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(predictor_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(predictor_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/predictor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(predictor_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(predictor_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(predictor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/predictor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(predictor_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(predictor_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(predictor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/predictor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/predictor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/predictor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(predictor_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(predictor_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(predictor_generate_messages_py std_msgs_generate_messages_py)
endif()
