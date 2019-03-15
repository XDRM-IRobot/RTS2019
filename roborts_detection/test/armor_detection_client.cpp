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
    if(enemy_pose_candidate_.size())
      enemy_pose_ = enemy_pose_candidate_[0];
  }
  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
    tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;

      enemy_pose_candidate_.clear();

      std::vector<geometry_msgs::PoseStamped>::const_iterator it = feedback->enemy_pos.begin(); 
      for (int i = 0; it != feedback->enemy_pos.end(); ++it, ++i)
      {
        camera_pose_msg = feedback->enemy_pos[i];

        double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
                                  camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
        double yaw      = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);
      
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
        camera_pose_msg.pose.orientation.w = quaternion.w();
        camera_pose_msg.pose.orientation.x = quaternion.x();
        camera_pose_msg.pose.orientation.y = quaternion.y();
        camera_pose_msg.pose.orientation.z = quaternion.z();
        poseStampedMsgToTF(camera_pose_msg, tf_pose);
        tf_pose.stamp_ = ros::Time(0);      
        try
        {
          tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
          tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

          if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
            enemy_pose_candidate_.push_back(global_pose_msg);
          }
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("tf error when transform enemy pose from camera to map");
        }
      }
  }
  void PubGimbalControl()
  {
    float pitch, yaw;
    cv::Point3f target_3d;
    target_3d.x = enemy_pose_.pose.position.x;
    target_3d.y = enemy_pose_.pose.position.y;
    target_3d.z = enemy_pose_.pose.position.z;
    gimbal_control_.Transform(target_3d, pitch, yaw);

    gimbal_angle_.yaw_mode = true;
    gimbal_angle_.pitch_mode = false;
    gimbal_angle_.yaw_angle = yaw * 0.7;
    gimbal_angle_.pitch_angle = pitch;

    enemy_info_pub_.publish(gimbal_angle_);
  }
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      GetEnemyGloalPose(feedback);
      SelectFinalEnemy();
      PubGimbalControl();
    } else{
      enemy_detected_ = false;
    }
  }

private:

  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  geometry_msgs::PoseStamped enemy_pose_;
  std::vector<geometry_msgs::PoseStamped> enemy_pose_candidate_;
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
