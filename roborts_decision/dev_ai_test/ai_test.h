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

#pragma once 

#include <ros/ros.h>
#include "io/io.h"
#include "state/error_code.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <gimbal_control.h>
#include "proto/gimbal_control.pb.h"

#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

#include "roborts_msgs/GlobalPlannerAction.h"
#include "roborts_msgs/LocalPlannerAction.h"

namespace roborts_decision{

using roborts_common::ErrorCode;

class AI_Test
{
public:
  AI_Test(): 
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true),
      global_planner_actionlib_client_("global_planner_node_action", true),
      local_planner_actionlib_client_("local_planner_node_action", true) // Added
      
  {
    init_ros();
    init_gimbal();
  }
  void init_ros()
  {
    ros_nh_                      = ros::NodeHandle();
    ros_ctrl_gimbal_angle_       = ros_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    ros_ctrl_fric_wheel_client_  = ros_nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros_ctrl_shoot_client_       = ros_nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    armor_detection_actionlib_client_.waitForServer();
    global_planner_actionlib_client_.waitForServer();

    ROS_INFO("Waiting for action server to start.");
  }
  void init_gimbal()
  {
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

    fric_wheel_.request.open  = param.fric_wheel();

    if(fric_wheel_.request.open)
    {
      ros_ctrl_fric_wheel_client_.call(fric_wheel_);
      usleep(3000000);  // wait 3s
    }
  }

  void SelectFinalEnemy();
  void ShootControl(float& yaw, float& pitch);
  void GimbalAngleControl(float& yaw, float& pitch);
  void NavGoalCallback(const geometry_msgs::PoseStamped & goal);
  void DoneCallback(const actionlib::SimpleClientGoalState& state,  
                    const roborts_msgs::GlobalPlannerResultConstPtr& result);
  void ActiveCallback();
  void FeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& feedback);

  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback);
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback);

  void start()
  {                     
    ROS_INFO("Start AI Test.");

    roborts_msgs::ArmorDetectionGoal goal;
    goal.command = 1;
    
    armor_detection_actionlib_client_.sendGoal(goal, 
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
            boost::bind(&AI_Test::ArmorDetectionCallback, this, _1));
  }   

private:
  
  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  geometry_msgs::Point enemy_pose_;
  std::vector<geometry_msgs::Point> ptz_point_candidate_;
  std::vector<geometry_msgs::PoseStamped> global_pose_candidate_;

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  tf::TransformBroadcaster tf_in_map_;
  //! anglesolver
  roborts_detection::GimbalContrl gimbal_control_;

  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Publisher     ros_ctrl_gimbal_angle_;
  ros::ServiceClient ros_ctrl_fric_wheel_client_;
  ros::ServiceClient ros_ctrl_shoot_client_;

  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::FricWhl     fric_wheel_;
  roborts_msgs::ShootCmd    shoot_cmd_;

  // nav
  roborts_msgs::GlobalPlannerGoal global_planner_goal_;
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> global_planner_actionlib_client_;

  roborts_msgs::LocalPlannerGoal local_planner_goal_;
  actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction>  local_planner_actionlib_client_;
};

}

