# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/noetic/include".split(';') if "${prefix}/include;/opt/ros/noetic/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;franka_control;dynamic_reconfigure;franka_hw;geometry_msgs;hardware_interface;message_runtime;pluginlib;realtime_tools;roscpp;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_aic;/opt/ros/noetic/lib/x86_64-linux-gnu/libfranka.so.0.9.2".split(';') if "-lfranka_aic;/opt/ros/noetic/lib/x86_64-linux-gnu/libfranka.so.0.9.2" != "" else []
PROJECT_NAME = "franka_aic"
PROJECT_SPACE_DIR = "/home/neurorobotic_student/panda-robot-control/panda_ws/install"
PROJECT_VERSION = "0.5.0"
