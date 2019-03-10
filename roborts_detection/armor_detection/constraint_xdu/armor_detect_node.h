/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _ARMOR_DETECT_NODE_H_
#define _ARMOR_DETECT_NODE_H_

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point32.h>

#include <armor_info.h>
#include <armor_detect.h>

#include <ros/ros.h>
#include <moss_msgs/car_info.h>
#include <moss_msgs/armor_info.h>

namespace detect_mul
{
class armor_detect_node
{
public:
    armor_detect_node()
    {
        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
        sub_img_   = it.subscribe("camera_info", 5, 
                        boost::bind(&armor_detect_node::armor_callback, this, _1));

        sub_yaw_   = n.subscribe<moss_msgs::car_info>("car_info",5, 
                        boost::bind(&armor_detect_node::car_callback, this, _1));

        pub_armor_ = n.advertise<moss_msgs::armor_info>("armor_info", 5);
    }

    void armor_callback(const sensor_msgs::ImageConstPtr& msg);
    void car_callback(const moss_msgs::car_info::ConstPtr &gimbal_info);

private:
    image_transport::Subscriber sub_img_;
    ros::Subscriber sub_yaw_;
    ros::Publisher  pub_armor_;

    moss_msgs::armor_info armor_info_;
    int detected;
    
    double gimbal_yaw_;
    double gimbal_pitch_;
    
    //ArmorDetector armor_detect_;
    //std::vector<armor_info> multi_armors_;
};

} // namespace detect_mul

#endif
