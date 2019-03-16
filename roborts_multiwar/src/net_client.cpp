#include <ros/ros.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "rm_net.h"
#include <roborts_msgs/BattleField.h>

// 订阅net，向决策发送

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<roborts_msgs::BattleField>("team_info", 5); // update data

    int udp_port = 8888;
    ros::param::get("udp_port",udp_port);

    namespace ip = boost::asio::ip;
    boost::asio::io_service io_service;
  
    // Client binds to any address on port 8888 (the same port on which
    // broadcast data is sent from server).
    ip::udp::socket socket(io_service, ip::udp::endpoint(ip::udp::v4(), udp_port ));
  
    ip::udp::endpoint sender_endpoint;
  
    // Receive data.
    uint8_t send_bytes[22];
    NetBuffer buf;

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        std::size_t bytes_transferred = socket.receive_from(boost::asio::buffer(send_bytes), sender_endpoint);
        if (udp_decode(buf, send_bytes, bytes_transferred))
        {
            std::cout << "remote host ip : " << sender_endpoint.address().to_string() <<"\t";
            std::cout << "got " << bytes_transferred << " bytes. " 
                << "      bullet: "<< buf.my_bullet_
                << "      HP: "<< buf.my_health_ 
                << "\n";
        }
        roborts_msgs::BattleField info;
        info.my_bullet     = buf.my_bullet_;
        info.my_health     = buf.my_health_;
        info.my_position.x = buf.my_position_x_;
        info.my_position.y = buf.my_position_y_; 

        info.enemy1_health     = buf.enemy1_health_;
        info.enemy1_position.y = buf.enemy1_position_y_; 
        info.enemy1_position.y = buf.enemy1_position_y_; 

        info.enemy2_health     = buf.enemy2_health_;
        info.enemy2_position.y = buf.enemy2_position_y_; 
        info.enemy2_position.y = buf.enemy2_position_y_; 
        pub.publish(info);
    
        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}