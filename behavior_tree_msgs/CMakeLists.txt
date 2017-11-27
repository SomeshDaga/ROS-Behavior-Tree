cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
    KeyValue.msg
)

add_service_files(
  FILES
    UpdateBlackboard.srv
)

generate_messages()

catkin_package(
 CATKIN_DEPENDS
   message_runtime
)
