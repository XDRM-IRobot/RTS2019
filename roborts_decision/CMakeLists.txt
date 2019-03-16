project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_subdirectory(dev_vision_test)

#
#add_executable(behavior_test_node
#  ${DecisionProtoHds}
#  ${DecisionProtoSrc}
#  behavior/escape_behavior.h
#  behavior/back_boot_area_behavior.h
#  behavior/chase_behavior.h
#  behavior/patrol_behavior.h
#  behavior/search_behavior.h
# behavior_test.cpp
#  executor/chassis_executor.cpp
#  )

#target_link_libraries(behavior_test_node
#  PRIVATE
#  roborts_costmap
#  ${catkin_LIBRARIES}
#  ${PROTOBUF_LIBRARIES}
#  )
#add_dependencies(behavior_test_node
#  roborts_msgs_generate_messages)
