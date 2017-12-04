cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_core)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  behavior_tree_msgs
  genmsg
  message_generation
  roscpp
  roslint
  rospy
  roslaunch
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS)

add_action_files(
  DIRECTORY action
  FILES BT.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs # Or other packages containing msgs
)


add_definitions(-Wall -lglut -lGL -std=c++11)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    behavior_tree_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

roslint_cpp()

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} include)

file(GLOB_RECURSE BTHeadLibrary include/*.h)

set(BTSrcLibrary
src/action_node.cpp
src/behavior_tree.cpp
src/condition_node.cpp
src/control_node.cpp
src/draw.cpp
src/exceptions.cpp
src/leaf_node.cpp
src/tick_engine.cpp
src/parallel_node.cpp
src/fallback_node.cpp
src/sequence_node.cpp
src/fallback_node_with_memory.cpp
src/sequence_node_with_memory.cpp
src/tree_node.cpp
src/actions/action_test_node.cpp
src/conditions/condition_test_node.cpp
src/actions/ros_action.cpp
src/conditions/ros_condition.cpp
src/blackboard/blackboard.cpp
src/dot_bt.cpp
)

# Compile the core library with name ${PROJECT_NAME}=behavior_tree_core
# You can create executables which target to this library for using BTs
add_library(${PROJECT_NAME} ${BTSrcLibrary} ${BTHeadLibrary})
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} 
  ${OPENGL_LIBRARIES} 
  ${GLUT_LIBRARY}
)

add_dependencies(${PROJECT_NAME} behavior_tree_core_generate_messages_cpp)

add_executable(tree src/tree.cpp)
target_link_libraries(tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

catkin_add_gtest(gtest_tree src/gtest/gtest_tree.cpp)
target_link_libraries(gtest_tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GTest_LIBRARIES}
)

add_executable(gtest_ros src/gtest/external_ros_nodes_test.cpp)
target_link_libraries(gtest_ros
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

