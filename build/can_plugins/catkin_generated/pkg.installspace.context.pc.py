# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "nodelet;pluginlib;roscpp;std_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcan_plugins".split(';') if "-lcan_plugins" != "" else []
PROJECT_NAME = "can_plugins"
PROJECT_SPACE_DIR = "/home/crs/mainmachine/install"
PROJECT_VERSION = "0.0.0"
