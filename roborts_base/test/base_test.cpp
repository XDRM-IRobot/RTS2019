/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>

#include "io/io.h"

#include "proto/ BaseControlParam.pb.h"

#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

class BaseTest
{
public:
  BaseTest()
  {
    ros_nh_                      = ros::NodeHandle();
    ros_ctrl_gimbal_angle_       = ros_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    ros_ctrl_fric_wheel_client_  = ros_nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros_ctrl_shoot_client_       = ros_nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    roborts_base::BaseControlParam param;
    std::string file_name = ros::package::getPath("roborts_base") + "/test/config/BaseControlParam.prototxt";
    bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &param);
    if (!read_state) {
      ROS_ERROR("Cannot open %s", file_name.c_str());
      //return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
    }
    
    shoot_mode_           = param.shoot_mode();
    shoot_number_         = param.shoot_number();
    gimbal_yaw_mode_      = param.gimbal_yaw_mode();
    gimbal_pitch_mode_    = param.gimbal_pitch_mode();
    gimbal_control_freq_  = param.gimbal_control_freq();
    gimbal_control_range_ = param.gimbal_control_range();

    gimbal_angle_.yaw_mode    = gimbal_yaw_mode_;
    gimbal_angle_.pitch_mode  = gimbal_pitch_mode_;

    fric_wheel_.request.open  = param.fric_wheel_open();

    ros_ctrl_fric_wheel_client_.call(fric_wheel_);

  }

  void run()
  {
    ros::Rate loop_rate(gimbal_control_freq_);

    long dt = 0;
    float yaw   = 0;
    float pitch = 0;

    while(ros::ok)
    {
      yaw = cos(0.05 * dt++) * gimbal_control_range_;
      pitch = 0;
      GimbalAngleControl(yaw, pitch);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void ShootControl(float& yaw, float& pitch)
  {
    if(!fric_wheel_.request.open)
    {
        fric_wheel_.request.open  = true;
        usleep(3000000);
    }
    shoot_cmd_.request.mode   = shoot_mode_;
    shoot_cmd_.request.number = shoot_number_;

    ros_ctrl_shoot_client_.call(shoot_cmd_);
  }

  void GimbalAngleControl(float& yaw, float& pitch)
  {
    gimbal_angle_.yaw_angle   = -yaw   * 180 / M_PI;
    gimbal_angle_.pitch_angle = pitch * 180 / M_PI;
    
    ros_ctrl_gimbal_angle_.publish(gimbal_angle_);
    ROS_ERROR("yaw = %f , pitch = %f ",gimbal_angle_.yaw_angle, gimbal_angle_.pitch_angle);
  }

private:
  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Publisher     ros_ctrl_gimbal_angle_;
  ros::ServiceClient ros_ctrl_fric_wheel_client_;
  ros::ServiceClient ros_ctrl_shoot_client_;

  // msg srv
  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::FricWhl     fric_wheel_;
  roborts_msgs::ShootCmd    shoot_cmd_;

  // param
  int shoot_mode_;
  int shoot_number_;
  int gimbal_yaw_mode_;
  int gimbal_pitch_mode_;
  int gimbal_control_freq_;
  float gimbal_control_range_;
};


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "armor_detection_node_test_client");
  BaseTest test;
  test.run();
  return 0;
}
