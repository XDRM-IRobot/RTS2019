/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robomaster Team, IRobot.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef ROBORTS_BASE_REFEREE_H
#define ROBORTS_BASE_REFEREE_H

#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {
/**
 * @brief ROS API for Referee module
 */
class Referee {
 public:
  /**
   * @brief Constructor of Referee including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  Referee(std::shared_ptr<roborts_sdk::Handle> handle);

  /**
   * @brief Destructor of Referee
   */
  ~Referee() = default;

 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();

  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**
   * @brief Referee information callback in sdk
   * @param Referee_info Referee information
   */
  void RefereeInfoCallback(const std::shared_ptr<roborts_sdk::cmd_referee_info> Referee_info);

  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;

  ros::NodeHandle ros_nh_;
  ros::Publisher  ros_pub_;

  roborts_msgs::Referee referee_info_; 

};
}
#endif //ROBORTS_BASE_REFEREE_H
