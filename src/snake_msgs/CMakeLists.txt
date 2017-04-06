cmake_minimum_required(VERSION 2.8.3)
project(snake_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   snake_joint_command.msg
   snake_joint_data.msg

 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snakebot_msgs
#  CATKIN_DEPENDS message_generation std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
