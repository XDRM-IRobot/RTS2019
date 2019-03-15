project(roborts_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
set(COMPILE_PROTO 0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  roborts_msgs
  roborts_common
  )

find_package(ProtoBuf REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${EIGEN3_INCLUDE_DIR} util test)

add_subdirectory(util)
add_subdirectory(armor_detection)




file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/test/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/test/proto
  GimbalControlProtoSrc
  GimbalControlProtoHds
  ${ProtoFiles})

add_executable(armor_detection_client
  test/gimbal_control.cpp
  test/armor_detection_client.cpp
  ${GimbalControlProtoSrc}
  ${GimbalControlProtoHds}
)

target_include_directories(armor_detection_client
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

target_link_libraries(armor_detection_client
  PRIVATE
  detection::tool
  detection::constraint_set
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(armor_detection_client
  roborts_msgs_generate_messages)

