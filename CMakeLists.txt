cmake_minimum_required(VERSION 2.8.3)
project(xfm_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  gpio_msg.msg
  xfm_cmd.msg
  xfm_status.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES xfm_driver
  CATKIN_DEPENDS message_runtime rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
