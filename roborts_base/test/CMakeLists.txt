project(test)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  BaseControlProtoSrc
  BaseControlProtoHds
  ${ProtoFiles})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_test
  base_test.cpp
  ${BaseControlProtoSrc}
  ${BaseControlProtoHds}
)

target_link_libraries(base_test 
  PUBLIC
  detection::tool
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )

add_dependencies(base_test roborts_msgs_generate_messages)

