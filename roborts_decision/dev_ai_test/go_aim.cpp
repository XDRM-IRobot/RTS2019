#include "ai_test.h"

namespace roborts_decision{

void AI_Test::SelectFinalEnemy()
  {
    if(ptz_point_candidate_.size())
      enemy_pose_ = ptz_point_candidate_[0];
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
}