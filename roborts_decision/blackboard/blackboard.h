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
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "io/io.h"
#include "../proto/decision.pb.h"

#include "costmap/costmap_interface.h"

#include "roborts_msgs/ArmorDetectionAction.h"
#include <roborts_msgs/GimbalAngle.h>
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/BattleField.h"  // multi-car

namespace roborts_decision{

class Blackboard {
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;

  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true)
  {
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if(decision_config.simulate())
    {
      // Enemy fake pose
      ros::NodeHandle rviz_nh("/move_base_simple");
      enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);
    }
    else{
      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
      
      battlefield_sub_ = ros_nh_.subscribe<roborts_msgs::BattleField>("other_call", 5, &Blackboard::BattleFieldCallback, this);
      battlefield_pub_ = ros_nh_.advertise<roborts_msgs::BattleField>("call_other", 5);
      
    }
  }

  ~Blackboard() = default;

  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
      ptz_point_candidate_.clear();
      global_pose_candidate_.clear();

      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = feedback->enemy_pos[i].x - 0;  // gimbal_control_.offset_.x;
        pt.y = feedback->enemy_pos[i].y - 10; // gimbal_control_.offset_.y;
        pt.z = feedback->enemy_pos[i].z + 1;  // gimbal_control_.offset_.z;
        ptz_point_candidate_.push_back(pt);

        geometry_msgs::PoseStamped ptz_pose;
        ptz_pose.header.stamp       = ros::Time();
        ptz_pose.header.frame_id    = "gimbal_link";
        ptz_pose.pose.position.x    = pt.x / 100.;
        ptz_pose.pose.position.y    = pt.z / 100.;
        ptz_pose.pose.position.z    = pt.y / 100.;

        float yaw = ptz_pose.pose.position.y / ptz_pose.pose.position.x;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);

        ptz_pose.pose.orientation.x = quaternion.x();
        ptz_pose.pose.orientation.y = quaternion.y();
        ptz_pose.pose.orientation.z = quaternion.z();
        ptz_pose.pose.orientation.w = quaternion.w();

        //tf::Transform transform
        try{
          geometry_msgs::PoseStamped map_pose;
          tf::Stamped<tf::Pose> map_tf_pose;
          tf_ptr_->transformPose("map", ptz_pose, map_pose);
          poseStampedMsgToTF(map_pose, map_tf_pose);
          tf_in_map_.sendTransform(tf::StampedTransform(map_tf_pose, ros::Time::now(), "map", "enemy_link"));
          global_pose_candidate_.push_back(map_pose);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("tf error when transform enemy pose from /gimbal_link to /map");
          ros::Duration(1.0).sleep();
        }
      }
  }
  void BattleFieldCallback(const roborts_msgs::BattleFieldConstPtr& Battlefield)
  {

  }
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      GetEnemyGloalPose(feedback);
      
      // SelectFinalEnemy();
      
      // float yaw, pitch;
      // GimbalAngleControl(yaw, pitch);
      // ShootControl(yaw, pitch);

    } else{
      enemy_detected_ = false;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
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

  //! 3D point in ptz
  std::vector<geometry_msgs::Point>       ptz_point_candidate_;   
  //! 3D pose in map
  std::vector<geometry_msgs::PoseStamped> global_pose_candidate_; 

  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  tf::TransformBroadcaster tf_in_map_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;
  ros::Subscriber battlefield_sub_;
  ros::Publisher  battlefield_pub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  std::vector<geometry_msgs::Point> pose_point_candidate_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  //! anglesolver
  //roborts_detection::GimbalContrl gimbal_control_;

  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Publisher     ros_ctrl_gimbal_angle_;
  ros::ServiceClient ros_ctrl_fric_wheel_client_;
  ros::ServiceClient ros_ctrl_shoot_client_;

  roborts_msgs::GimbalAngle gimbal_angle_;
  roborts_msgs::FricWhl     fric_wheel_;
  roborts_msgs::ShootCmd    shoot_cmd_;

};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
