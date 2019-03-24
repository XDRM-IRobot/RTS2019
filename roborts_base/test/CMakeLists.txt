project(test)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_test
  base_test.cpp
)

target_link_libraries(base_test 
  PUBLIC
  ${catkin_LIBRARIES}
  )

add_dependencies(base_test roborts_msgs_generate_messages)
