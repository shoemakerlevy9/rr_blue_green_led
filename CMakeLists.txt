cmake_minimum_required(VERSION 2.8.3)
project(rr_blue_green_led)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES led_control
   CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  scripts/arduino_led_node.py
  scripts/test_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_led_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
