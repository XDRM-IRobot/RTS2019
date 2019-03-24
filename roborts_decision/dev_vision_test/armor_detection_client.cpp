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
#include "roborts_msgs/ArmorDetectionAction.h"
#include <actionlib/client/terminal_state.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <gimbal_control.h>
#include <roborts_msgs/GimbalAngle.h>

#include "proto/gimbal_control.pb.h"

namespace roborts_decision{

class ArmorBehavier
{
public:
  ArmorBehavier(): 
  armor_detection_actionlib_client_("armor_detection_node_action", true),
  enemy_detected_(false)
  {
    enemy_nh_       = ros::NodeHandle();
    enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);

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
  void SelectFinalEnemy()
  {
    if(pose_point_candidate_.size())
      enemy_pose_ = pose_point_candidate_[0];
  }

  void GimbalAngleControl(geometry_msgs::Point pt)
  {
    if(enemy_detected_)
    {
      cv::Point3f target;
      target.x = pt.x;
      target.y = pt.y;
      target.z = pt.z;

      float yaw, pitch;
      gimbal_control_.SolveContrlAgnle(target, yaw, pitch);
      gimbal_angle_.yaw_angle   = -yaw;
      gimbal_angle_.pitch_angle = pitch;

      enemy_info_pub_.publish(gimbal_angle_);

      ROS_ERROR("yaw = %f , pitch = %f ",gimbal_angle_.yaw_angle,gimbal_angle_.pitch_angle);
    }
    else{
      gimbal_angle_.yaw_angle   = 0;
      gimbal_angle_.pitch_angle = 0;
      enemy_info_pub_.publish(gimbal_angle_);
      ROS_ERROR("yaw = %f , pitch = %f ",gimbal_angle_.yaw_angle,gimbal_angle_.pitch_angle);
    }
  }
  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::Point pose_point;

      pose_point_candidate_.clear();

      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        pose_point = feedback->enemy_pos[i];
        // transform camera to ptz
        pose_point.x += gimbal_control_.offset_.x; 
        pose_point.y += gimbal_control_.offset_.y;
        pose_point.z += gimbal_control_.offset_.z;
        pose_point_candidate_.push_back(pose_point);
      }
      GimbalAngleControl(pose_point);
  }
  
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      GetEnemyGloalPose(feedback);

    } else{
      enemy_detected_ = false;
    }
  }

private:

  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  geometry_msgs::Point enemy_pose_;
  std::vector<geometry_msgs::Point> pose_point_candidate_;
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! control model
  ros::NodeHandle enemy_nh_;
  ros::Publisher enemy_info_pub_;
  roborts_detection::GimbalContrl gimbal_control_;
  roborts_msgs::GimbalAngle gimbal_angle_;

private:

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) 
  {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) 
  {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

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
