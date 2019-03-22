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
#include "io/io.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <gimbal_control.h>
#include "proto/gimbal_control.pb.h"

#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

namespace roborts_decision{

class ArmorBehavier
{
public:
  ArmorBehavier(): 
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      enemy_detected_(false)
  {
    ros_nh_                      = ros::NodeHandle();
    ros_ctrl_gimbal_angle_       = ros_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    ros_ctrl_fric_wheel_client_  = ros_nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros_ctrl_shoot_client_       = ros_nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    armor_detection_actionlib_client_.waitForServer();
    ROS_INFO("Waiting for action server to start.");

    roborts_decision::GimbalControlParam param;
    std::string file_name = ros::package::getPath("roborts_detection") + "/test/config/gimbal_control.prototxt";
    bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &param);
    if (!read_state) {
      ROS_ERROR("Cannot open %s", file_name.c_str());
      //return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
    }
    gimbal_control_.Init(param.offset_x(),
                         param.offset_y(),
                         param.offset_z(),
                         param.offset_pitch(),
                         param.offset_yaw(), 
                         param.init_v(),
                         param.init_k());

    gimbal_angle_.yaw_mode    = true;
    gimbal_angle_.pitch_mode  = true;

    //fric_wheel_.request.open  = true;
    //ros_ctrl_fric_wheel_client_.call(fric_wheel_);

  }

  void start()
  {
    ROS_INFO("Start.");

    roborts_msgs::ArmorDetectionGoal goal;
    goal.command = 1;

    ROS_INFO("I am running the request");

    armor_detection_actionlib_client_.sendGoal(goal, 
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
            boost::bind(&ArmorBehavier::ArmorDetectionCallback, this, _1));
  }

private:
  void ShootControl(float& yaw, float& pitch)
  {
    if(fric_wheel_.request.open)
    {
      if (abs(yaw) < 2 &&  abs(pitch) < 2)
      {
        shoot_cmd_.request.mode   = true;
        shoot_cmd_.request.number = true;
        ros_ctrl_shoot_client_.call(shoot_cmd_);
      }
    }
  }

  void GimbalAngleControl(float& yaw, float& pitch)
  {
    if(enemy_detected_)
    {
      gimbal_control_.SolveContrlAgnle(enemy_pose_, yaw, pitch);

      double dyaw   = 0.1;
      double dpitch = 0.1;

           if (abs(yaw) > 30) dyaw = 1;
      else if (abs(yaw) > 25) dyaw = 0.8;
      else if (abs(yaw) > 20) dyaw = 0.7;
      else if (abs(yaw) > 15) dyaw = 0.6;
      else if (abs(yaw) > 10) dyaw = 0.5;
      else if (abs(yaw) > 9)  dyaw = 0.45;
      else if (abs(yaw) > 8)  dyaw = 0.4;
      else if (abs(yaw) > 7)  dyaw = 0.35;
      else if (abs(yaw) > 6)  dyaw = 0.3;
      else if (abs(yaw) > 5)  dyaw = 0.25;
      else if (abs(yaw) > 5)  dyaw = 0.2;
      else if (abs(yaw) > 5)  dyaw = 0.175;
      else if (abs(yaw) > 5)  dyaw = 0.15;
      else if (abs(yaw) > 5)  dyaw = 0.125;
      else                    dyaw = 0.1;

      gimbal_angle_.yaw_angle   = -yaw  * dyaw;
      gimbal_angle_.pitch_angle = pitch * dpitch;

      ros_ctrl_gimbal_angle_.publish(gimbal_angle_);
      ROS_ERROR("yaw = %f , pitch = %f , dyaw = %f , dpitch = %f  ",
                gimbal_angle_.yaw_angle, 
                gimbal_angle_.pitch_angle, 
                dyaw, 
                dpitch);
    }
    else{
      gimbal_angle_.yaw_angle   = 0;
      gimbal_angle_.pitch_angle = 0;
      ros_ctrl_gimbal_angle_.publish(gimbal_angle_);
      ROS_ERROR("yaw = %f , pitch = %f ",gimbal_angle_.yaw_angle,gimbal_angle_.pitch_angle);
    }
  }
 
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      ptz_point_candidate_.clear();
      
      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = feedback->enemy_pos[i].x - gimbal_control_.offset_.x;
        pt.y = feedback->enemy_pos[i].y - gimbal_control_.offset_.y;
        pt.z = feedback->enemy_pos[i].z - gimbal_control_.offset_.z;
        ptz_point_candidate_.push_back(pt);
      }

      if(ptz_point_candidate_.size())
        enemy_pose_ = ptz_point_candidate_[0];
      
      float yaw, pitch;
      GimbalAngleControl(yaw, pitch);
      //ShootControl(yaw, pitch);

    } else{
      enemy_detected_ = false;
      ROS_INFO("no Enemy!");
    }
  }

private:
  
  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  geometry_msgs::Point enemy_pose_;
  std::vector<geometry_msgs::Point> ptz_point_candidate_;

  roborts_detection::GimbalContrl gimbal_control_;
  roborts_msgs::GimbalAngle gimbal_angle_;

  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Publisher     ros_ctrl_gimbal_angle_;
  ros::ServiceClient ros_ctrl_fric_wheel_client_;
  ros::ServiceClient ros_ctrl_shoot_client_;

  roborts_msgs::FricWhl fric_wheel_;
  roborts_msgs::ShootCmd shoot_cmd_;
};

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "armor_detection_node_test_client");
  roborts_decision::ArmorBehavier ac;
  ac.start();
  ros::spin();
  return 0;
}
