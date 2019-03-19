
project(dev_ai_test)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  GimbalControlProtoSrc
  GimbalControlProtoHds
  ${ProtoFiles})

add_executable(ai_test
  gimbal_control.cpp
  ai_test.cpp
  ${GimbalControlProtoSrc}
  ${GimbalControlProtoHds}
)

target_include_directories(ai_test
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

target_link_libraries(ai_test
  PRIVATE
  detection::tool
  detection::constraint_set
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(ai_test
  roborts_msgs_generate_messages)

