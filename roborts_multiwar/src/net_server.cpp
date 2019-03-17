#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <iostream>

#include "rm_net.h"
#include <roborts_msgs/BattleField.h>

using namespace std;

NetBuffer buffer;

void BattleCallBack(const roborts_msgs::BattleField& info) // 把我的信息发出去
{
    buffer.frame_head_ = 0xEE;
    buffer.frame_tail_ = 0xEF;
    buffer.car_id_     = 1;

    buffer.my_bullet_ = info.my_bullet;

    buffer.my_health_     = info.my_health;
    buffer.my_position_x_ = info.my_position.x;
    buffer.my_position_y_ = info.my_position.y;

    buffer.enemy1_health_     = info.enemy1_health;
    buffer.enemy1_position_x_ = info.enemy1_position.x;
    buffer.enemy1_position_y_ = info.enemy1_position.y;

    buffer.enemy2_health_     = info.enemy2_health;
    buffer.enemy2_position_x_ = info.enemy2_position.x;
    buffer.enemy2_position_y_ = info.enemy2_position.y;
}

// 订阅决策，向net发送

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_publisher");
    ros::NodeHandle nh;
    ros::Subscriber sub_vision = nh.subscribe("my_info", 5, BattleCallBack); // update data

    int udp_port = 8888;
    int udp_rate = 10;
    ros::param::get("udp_port",udp_port);
    ros::param::get("udp_rate",udp_rate);

    ros::Rate loop_rate(udp_rate);

    namespace ip = boost::asio::ip;
    boost::asio::io_service io_service;
  
    // Server binds to any address and any port.
    ip::udp::socket socket(io_service, ip::udp::endpoint(ip::udp::v4(), 0));
    socket.set_option(boost::asio::socket_base::broadcast(true));
  
    // Broadcast will go to port 8888.
    ip::udp::endpoint broadcast_endpoint(ip::address_v4::broadcast(), udp_port);
  
    // Broadcast data.
    uint8_t send_bytes[22];

    udp_encode(buffer, send_bytes);

    while(ros::ok())
    {
        socket.send_to(boost::asio::buffer(send_bytes), broadcast_endpoint);
        ros::spinOnce();
        loop_rate.sleep();
    }
    io_service.run();

    return EXIT_SUCCESS;
}