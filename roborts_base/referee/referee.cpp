/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robomaster Team, IRobot.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "referee.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base{
Referee::Referee(std::shared_ptr<roborts_sdk::Handle> handle):
    handle_(handle){
  SDK_Init();
  ROS_Init();
}

void Referee::SDK_Init(){
  handle_->CreateSubscriber<roborts_sdk::cmd_referee_info>(CHASSIS_CMD_SET, CMD_PUSH_REFEREE_INFO,
                                                           CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                           std::bind(&Referee::RefereeInfoCallback, this, std::placeholders::_1));
}

void Referee::ROS_Init()
{
  ros_pub_ = ros_nh_.advertise<roborts_msgs::Referee>("referee", 30);
}

void Referee::RefereeInfoCallback(const std::shared_ptr<roborts_sdk::cmd_referee_info> Referee_info)
{
  ros::Time current_time = ros::Time::now();
  referee_info_.health = Referee_info->health;
  ros_pub_.publish(referee_info_);
}

}
