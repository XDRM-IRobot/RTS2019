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

#include "roborts_msgs/GimbalInfo.h"

#include <thread>
#include <condition_variable>

namespace roborts_decision{

using roborts_common::ErrorCode;

class AI_Test
{
public:
  AI_Test(): 
      enemy_detected_(false),
      lost_cnt_(0),
      detect_cnt_(0),
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
    ros_sub_gimbal_              = ros_nh_.subscribe("gimbal_info", 1, &AI_Test::ListenYaw, this);

    ros_ctrl_gimbal_angle_       = ros_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    ros_ctrl_fric_wheel_client_  = ros_nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    ros_ctrl_shoot_client_       = ros_nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    armor_detection_actionlib_client_.waitForServer();
    global_planner_actionlib_client_.waitForServer();

    ROS_INFO("Waiting for action server to start.");

    ROS_INFO("Start AI Test.");

    roborts_msgs::ArmorDetectionGoal goal;
    goal.command = 1;
    
    armor_detection_actionlib_client_.sendGoal(goal, 
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
            actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
            boost::bind(&AI_Test::ArmorDetectionCallback, this, _1));

  }
  void init_gimbal()
  {
    roborts_decision::GimbalControlParam param;
    std::string file_name = ros::package::getPath("roborts_decision") + "/dev_ai_test/config/gimbal_control.prototxt";
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

    fric_wheel_.request.open  = param.fric_wheel();

    if(fric_wheel_.request.open)
    {
      ros_ctrl_fric_wheel_client_.call(fric_wheel_);
      usleep(3000000);  // wait 3s
    }
  }
  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  int SelectFinalEnemy();
  
  void ShootControl(float& yaw, float& pitch);
  void GimbalAngleControl(float& yaw, float& pitch);
  void NavGoalCallback(const geometry_msgs::PoseStamped & goal);
  void DoneCallback(const actionlib::SimpleClientGoalState& state,  
                    const roborts_msgs::GlobalPlannerResultConstPtr& result);
  void ActiveCallback();
  void FeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& feedback);

  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback);
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback);

  void start();
  void GetEnemyNavGoal(geometry_msgs::PoseStamped& nav, const float distance);
  void ExecuteLoop();
private:
  std::mutex mutex_;
  std::thread execute_thread_;// = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  int lost_cnt_;   
  int detect_cnt_;   

  std::vector<geometry_msgs::Point> shoot_candidate_;         // for decision
  std::vector<geometry_msgs::PoseStamped> enemy_candidate_;   // for decision
  geometry_msgs::Point shoot_target_;    // in ptz

  std::vector<geometry_msgs::PoseStamped> enemy_pose_in_ptz_; // for nav
  geometry_msgs::PoseStamped nav_target_;                      // for nav
  

  geometry_msgs::PoseStamped nav_goal_;  // in map
  geometry_msgs::PoseStamped robot_map_pose_;

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  tf::TransformBroadcaster tf_in_map_;
  tf::TransformListener tf_listener_;

  //! anglesolver
  roborts_detection::GimbalContrl gimbal_control_;

  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Subscriber    ros_sub_gimbal_;
  ros::Publisher     ros_ctrl_gimbal_angle_;
  ros::ServiceClient ros_ctrl_fric_wheel_client_;
  ros::ServiceClient ros_ctrl_shoot_client_;

  roborts_msgs::GimbalInfo  gimbal_info_;
  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::FricWhl     fric_wheel_;
  roborts_msgs::ShootCmd    shoot_cmd_;

  // nav
  roborts_msgs::GlobalPlannerGoal global_planner_goal_;
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> global_planner_actionlib_client_;

  roborts_msgs::LocalPlannerGoal local_planner_goal_;
  actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction>  local_planner_actionlib_client_;

  void ListenYaw(const roborts_msgs::GimbalInfo::ConstPtr &msg)
  {
    gimbal_info_.mode             = msg->mode;
    gimbal_info_.pitch_ecd_angle  = msg->pitch_ecd_angle;
    gimbal_info_.yaw_ecd_angle    = msg->yaw_ecd_angle;
    gimbal_info_.pitch_gyro_angle = msg->pitch_gyro_angle;
    gimbal_info_.yaw_gyro_angle   = msg->yaw_gyro_angle;
    gimbal_info_.yaw_rate         = msg->yaw_rate;
    gimbal_info_.pitch_rate       = msg->pitch_rate;

    /*ROS_ERROR("gimbal_info_.mode: %d ", gimbal_info_.mode );
    
    ROS_ERROR("gimbal_info_.pitch_ecd_angle: %f ",  gimbal_info_.pitch_ecd_angle);
    ROS_ERROR("gimbal_info_.yaw_ecd_angle: %f ",    gimbal_info_.yaw_ecd_angle );
    ROS_ERROR("gimbal_info_.pitch_gyro_angle: %f ", gimbal_info_.pitch_gyro_angle);
    ROS_ERROR("gimbal_info_.yaw_gyro_angle: %f ",   gimbal_info_.yaw_gyro_angle);
    ROS_ERROR("gimbal_info_.yaw_rate: %f ",         gimbal_info_.yaw_rate);
    ROS_ERROR("gimbal_info_.pitch_rate: %f ",       gimbal_info_.pitch_rate);
    */
  }
};

}

