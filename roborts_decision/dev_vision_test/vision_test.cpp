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
#include <tf/transform_broadcaster.h>
#include "proto/gimbal_control.pb.h"

namespace roborts_decision{

class ArmorBehavier
{
public:
  ArmorBehavier(): 
  armor_detection_actionlib_client_("armor_detection_node_action", true),
  enemy_detected_(false)
  {
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    ros_nh_           = ros::NodeHandle();
    enemy_info_pub_   = ros_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
    //pub_tf_in_camera_ = ros_nh_.advertise<tf::Stamped<tf::Pose>>("tf_in_camera", 100);
    //pub_tf_in_map_    = ros_nh_.advertise<tf::Stamped<tf::Pose>>("tf_in_map", 100);

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
    if(camera_pose_candidate_.size())
      enemy_pose_ = camera_pose_candidate_[0];
  }

  void GimbalAngleControl(geometry_msgs::Point target, float& yaw, float& pitch)
  {
    if(enemy_detected_)
    {
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

  void PoesStampedToCvPoint3f(geometry_msgs::Point& ros_point, cv::Point3f& cv_point)
  {
      cv_point.x = ros_point.x;
      cv_point.y = ros_point.y;
      cv_point.z = ros_point.z;
  }

  void CvPoint3fToPoesStamped(cv::Point3f& cv_point, geometry_msgs::Point& ros_point)
  {
      ros_point.x = cv_point.x;
      ros_point.x = cv_point.x;
      ros_point.x = cv_point.x;
  }

  void GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
      camera_pose_candidate_.clear();
      global_pose_candidate_.clear();

      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        geometry_msgs::PoseStamped camera_pose;
      
        camera_pose.header.stamp       = ros::Time();
        camera_pose.header.frame_id    = "camera_link";
        camera_pose.pose.position.x    = feedback->enemy_pos[i].x;
        camera_pose.pose.position.y    = feedback->enemy_pos[i].y;
        camera_pose.pose.position.z    = feedback->enemy_pos[i].z;

        float yaw = camera_pose.pose.position.z / camera_pose.pose.position.x;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);

        camera_pose.pose.orientation.x = quaternion.x();
        camera_pose.pose.orientation.y = quaternion.y();
        camera_pose.pose.orientation.z = quaternion.z();
        camera_pose.pose.orientation.w = quaternion.w();

        camera_pose_candidate_.push_back(camera_pose);

        // rviz
        camera_pose.pose.position.x    = camera_pose.pose.position.x /100.;
        camera_pose.pose.position.y    = camera_pose.pose.position.z /100.;
        camera_pose.pose.position.z    = camera_pose.pose.position.y /100.;

        tf::Stamped<tf::Pose> tf_pose;
        poseStampedMsgToTF(camera_pose, tf_pose);
        tf_in_camera_.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "camera_link", "enemy"));
        //tf::Transform transform
        try{
          geometry_msgs::PoseStamped map_pose;
          tf_ptr_->transformPose("map", camera_pose, map_pose);

          tf::Stamped<tf::Pose> map_tf_pose;
          poseStampedMsgToTF(camera_pose, map_tf_pose);
          tf_in_map_.sendTransform(tf::StampedTransform(map_tf_pose, ros::Time::now(), "map", "map_enemy"));

          global_pose_candidate_.push_back(map_pose);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("tf error when transform enemy pose from camera to map");
          ros::Duration(1.0).sleep();
        }
      }       
  }
  
  void ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      GetEnemyGloalPose(feedback);
      SelectFinalEnemy();
      
      float yaw, pitch;
      geometry_msgs::Point target = enemy_pose_.pose.position;
      GimbalAngleControl(target, yaw, pitch);
    } else{
      enemy_detected_ = false;
    }
  }

private:
  ros::NodeHandle ros_nh_;
  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  
  bool enemy_detected_;
  geometry_msgs::PoseStamped enemy_pose_;
  std::vector<geometry_msgs::PoseStamped> camera_pose_candidate_;
  std::vector<geometry_msgs::PoseStamped> global_pose_candidate_;
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;
  tf::TransformBroadcaster tf_in_camera_;
  tf::TransformBroadcaster tf_in_map_;

  //! control model
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
