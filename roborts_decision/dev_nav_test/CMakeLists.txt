
project(dev_nav_test)

add_executable(nav_test
  nav_test.cpp)

target_link_libraries(nav_test
  PRIVATE
  ${catkin_LIBRARIES}
)
add_dependencies(nav_test
  roborts_msgs_generate_messages)