
project(dev_vision_test)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  GimbalControlProtoSrc
  GimbalControlProtoHds
  ${ProtoFiles})

add_executable(armor_detection_client
  gimbal_control.cpp
  vision_test.cpp
  ${GimbalControlProtoSrc}
  ${GimbalControlProtoHds}
)

target_include_directories(armor_detection_client
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${CMAKE_CURRENT_SOURCE_DIR}
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

