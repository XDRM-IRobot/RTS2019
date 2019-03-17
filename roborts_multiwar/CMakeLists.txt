cmake_minimum_required(VERSION 3.1)
project(roborts_multiwar)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
    roborts_common
    roborts_msgs
)

catkin_package(CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    std_msgs
    roborts_msgs 
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
)

add_executable(udp_server src/net_server.cpp)
add_executable(udp_client src/net_client.cpp)

target_link_libraries(udp_server ${catkin_LIBRARIES} )
target_link_libraries(udp_client ${catkin_LIBRARIES} )

add_dependencies(udp_server 
  roborts_msgs_generate_messages)

add_dependencies(udp_client
  roborts_msgs_generate_messages)