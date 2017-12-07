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
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS)

add_action_files(
  DIRECTORY action
  FILES BT.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

add_definitions(-std=c++11)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    behavior_tree_msgs
    message_runtime
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

roslint_cpp()

file(GLOB_RECURSE BTHeadLibrary include/*.h
  EXCLUDE
    include/draw.h
)

set(BTSrcLibrary
src/action_node.cpp
src/actions/action_test_node.cpp
src/actions/ros_action.cpp
src/behavior_tree.cpp
src/blackboard/blackboard.cpp
src/condition_node.cpp
src/conditions/condition_test_node.cpp
src/conditions/ros_condition.cpp
src/control_node.cpp
src/dot_bt.cpp
src/exceptions.cpp
src/fallback_node.cpp
src/fallback_node_with_memory.cpp
src/leaf_node.cpp
src/parallel_node.cpp
src/sequence_node.cpp
src/sequence_node_with_memory.cpp
src/tick_engine.cpp
src/tree_node.cpp
)

#########################################################
# FIND GLUT AND OPENGL
#########################################################
find_package(GLUT)
if(NOT GLUT_FOUND)
    message("GLUT not found!")
endif()

find_package(OpenGL)
if(NOT OPENGL_FOUND)
    message("OPENGL not found!")
endif()

if(GLUT_FOUND AND OPENGL_FOUND)
  include_directories(${GLUT_INCLUDE_DIR})
  include_directories(${OPENGL_INCLUDE_DIR})

  set(BTHeadLibrary ${BTHeadLibrary} include/draw.h)
  set(BTSrcLibrary ${BTSrcLibrary} src/draw.cpp)
  list(SORT BTHeadLibrary)
  list(SORT BTSrcLibrary)

  add_library(${PROJECT_NAME}_lib ${BTHeadLibrary} ${BTSrcLibrary})
  target_link_libraries(${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES} 
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
  )
else()
  add_library(${PROJECT_NAME}_lib ${BTHeadLibrary} ${BTSrcLibrary})
  target_link_libraries(${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES} 
  )
endif()

include_directories(${catkin_INCLUDE_DIRS} include)


# Compile the core library with name ${PROJECT_NAME}=behavior_tree_core
# You can create executables which target to this library for using BTs

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}_generate_messages_cpp)

add_executable(tree src/tree.cpp)
target_link_libraries(tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_lib tree
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(gtest_tree src/gtest/gtest_tree.cpp)
  target_link_libraries(gtest_tree
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
    ${GTest_LIBRARIES}
  )

  add_executable(${PROJECT_NAME}_gtest_ros src/gtest/external_ros_nodes_test.cpp)
  target_link_libraries(${PROJECT_NAME}_gtest_ros
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
  )
endif()
