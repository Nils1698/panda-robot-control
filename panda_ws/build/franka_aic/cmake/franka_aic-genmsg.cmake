# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "franka_aic: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifranka_aic:/home/neurorobotic_student/panda_ws/src/franka_aic/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(franka_aic_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_custom_target(_franka_aic_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_aic" "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(franka_aic
  "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_aic
)

### Generating Services

### Generating Module File
_generate_module_cpp(franka_aic
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_aic
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(franka_aic_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(franka_aic_generate_messages franka_aic_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_aic_generate_messages_cpp _franka_aic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_aic_gencpp)
add_dependencies(franka_aic_gencpp franka_aic_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_aic_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(franka_aic
  "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_aic
)

### Generating Services

### Generating Module File
_generate_module_eus(franka_aic
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_aic
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(franka_aic_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(franka_aic_generate_messages franka_aic_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_aic_generate_messages_eus _franka_aic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_aic_geneus)
add_dependencies(franka_aic_geneus franka_aic_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_aic_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(franka_aic
  "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_aic
)

### Generating Services

### Generating Module File
_generate_module_lisp(franka_aic
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_aic
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(franka_aic_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(franka_aic_generate_messages franka_aic_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_aic_generate_messages_lisp _franka_aic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_aic_genlisp)
add_dependencies(franka_aic_genlisp franka_aic_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_aic_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(franka_aic
  "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_aic
)

### Generating Services

### Generating Module File
_generate_module_nodejs(franka_aic
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_aic
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(franka_aic_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(franka_aic_generate_messages franka_aic_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_aic_generate_messages_nodejs _franka_aic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_aic_gennodejs)
add_dependencies(franka_aic_gennodejs franka_aic_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_aic_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(franka_aic
  "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_aic
)

### Generating Services

### Generating Module File
_generate_module_py(franka_aic
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_aic
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(franka_aic_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(franka_aic_generate_messages franka_aic_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neurorobotic_student/panda_ws/src/franka_aic/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(franka_aic_generate_messages_py _franka_aic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_aic_genpy)
add_dependencies(franka_aic_genpy franka_aic_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_aic_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_aic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_aic
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_aic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_aic
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_aic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_aic
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_aic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_aic
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_aic)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_aic\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_aic
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
