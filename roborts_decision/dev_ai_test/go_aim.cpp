#include "ai_test.h"

namespace roborts_decision{


int AI_Test::SelectFinalEnemy()
{
    if(shoot_candidate_.size())
    {
      //...
      shoot_target_ = shoot_candidate_[0];
      return 0; 
      
    }
    /*try{
          geometry_msgs::PoseStamped ptz_pose_in_map;
          tf::Stamped<tf::Pose> ptz_pose_tf;
          tf_ptr_->transformPose("map", ptz_pose, ptz_pose_in_map);
          poseStampedMsgToTF(ptz_pose_in_map, ptz_pose_tf);
          tf_in_map_.sendTransform(tf::StampedTransform(ptz_pose_tf, ros::Time::now(), "map", "enemy_link"));
          global_pose_candidate_.push_back(map_pose);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("tf error when transform enemy pose from /gimbal_link to /map");
          ros::Duration(1.0).sleep();
        }
    */
}



  void AI_Test::ShootControl(float& yaw, float& pitch)
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

  void AI_Test::GimbalAngleControl(float& yaw, float& pitch)
  {
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
      //ROS_ERROR("yaw = %f , pitch = %f , dyaw = %f , dpitch = %f  ",
      //          gimbal_angle_.yaw_angle, 
      //          gimbal_angle_.pitch_angle, 
      //          dyaw, 
      //          dpitch);
  }
}