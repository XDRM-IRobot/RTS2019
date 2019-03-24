project(apriltags_detection)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  "${CMAKE_CURRENT_SOURCE_DIR}/AprilTags"
)

file(GLOB SrcFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

add_executable(apriltags_detection_node 
    ${SrcFiles}
    apriltags_detection.cpp
)

target_link_libraries(apriltags_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(apriltags_detection_node
  roborts_msgs_generate_messages)
