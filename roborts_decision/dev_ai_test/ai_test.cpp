
#include "ai_test.h"

namespace roborts_decision{
    
    unsigned long dt = 0;

  void AI_Test::GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
      shoot_candidate_.clear();
      enemy_pose_in_ptz_.clear();
      enemy_candidate_.clear();

      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        // gimbal
        geometry_msgs::Point pt;
        pt.x = feedback->enemy_pos[i].x - gimbal_control_.offset_.x;
        pt.y = feedback->enemy_pos[i].y - gimbal_control_.offset_.y;
        pt.z = feedback->enemy_pos[i].z - gimbal_control_.offset_.z;
        shoot_candidate_.push_back(pt);

        // nav
        geometry_msgs::PoseStamped ptz_pose;
        ptz_pose.header.stamp       = ros::Time();
        ptz_pose.header.frame_id    = "gimbal_link";
        ptz_pose.pose.position.x    =  pt.z / 100.;
        ptz_pose.pose.position.y    = -pt.x / 100.;
        ptz_pose.pose.position.z    = -pt.y / 100.;

        float yaw = ptz_pose.pose.position.y / ptz_pose.pose.position.x;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);

        ptz_pose.pose.orientation.x = quaternion.x();
        ptz_pose.pose.orientation.y = quaternion.y();
        ptz_pose.pose.orientation.z = quaternion.z();
        ptz_pose.pose.orientation.w = quaternion.w();

        enemy_pose_in_ptz_.push_back(ptz_pose);

        try{
          //map
          geometry_msgs::PoseStamped enemy_in_map;
          tf_ptr_->transformPose("map", ptz_pose, enemy_in_map);
          enemy_candidate_.push_back(enemy_in_map);

              //rviz
              tf::Stamped<tf::Pose> show_tf;
              poseStampedMsgToTF(enemy_in_map, show_tf);
              tf_in_map_.sendTransform(tf::StampedTransform(show_tf, ros::Time::now(), "map", "enemy_link"));
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("tf error when transform enemy pose from /gimbal_link to /map");
          ros::Duration(1.0).sleep();
        }
      }
  }
  
  void AI_Test::ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
    if (feedback->detected){
      if(detect_cnt_++ > 3)
      {
        //ROS_INFO("Find Enemy!");
        GetEnemyGloalPose(feedback);
        lost_cnt_ = 0;
        enemy_detected_ = true;
      }
    }
    else{
      if (lost_cnt_++ > 100)
      {
        //ROS_ERROR("no armor");
        detect_cnt_ = 0;
        enemy_detected_ = false;
      }
    }
  }

void AI_Test::ExecuteLoop()
{
  ros::Rate loop_rate(100);
  
  while(ros::ok())
    {
      //ROS_ERROR("loop thread.");
      if(enemy_detected_ && shoot_candidate_.size())
      {
        ROS_ERROR("enemy_detected_ && shoot_candidate_.size()");
        gimbal_angle_.yaw_mode    = true;
        gimbal_angle_.pitch_mode  = true;
         
        float yaw, pitch;
        
        int idx = SelectFinalEnemy();
        gimbal_control_.SolveContrlAgnle(shoot_target_, yaw, pitch);
        GimbalAngleControl(yaw, pitch);         
        
        tf::StampedTransform gimbal_tf;
        tf::Quaternion q;

        try{
          tf_listener_.lookupTransform("gimbal_link", "base_link", ros::Time(0), gimbal_tf);
          q = gimbal_tf.getRotation();
    
          double y, p, r;
          tf::Matrix3x3 m;
          m.setRotation(q);
          m.getEulerZYX( y, p, r);
          y = y * 180 / M_PI;
          p = p * 180 / M_PI;
          r = r * 180 / M_PI;

          //ROS_ERROR("listen tf from gimbal : yaw = %f, pitch = %f, roll = %f ", y, p, r);

        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
        }

        // nav
        if(nav_target_.pose.position.x > 3)
        {
          GetEnemyNavGoal(nav_target_, 3);
          ROS_INFO("Get nav goal.");
          global_planner_goal_.goal = nav_target_;
          global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                                    boost::bind(&AI_Test::DoneCallback, this, _1, _2),
                                                    boost::bind(&AI_Test::ActiveCallback, this),
                                                    boost::bind(&AI_Test::FeedbackCallback, this, _1));
        }
        continue;
      }
      else{
        //ROS_ERROR("enemy not found.");
         //global_planner_actionlib_client_.cancelGoal(); // no enemy stay here
         //local_planner_actionlib_client_.cancelGoal();

        gimbal_angle_.yaw_mode    = false;
        gimbal_angle_.pitch_mode  = false;

        float yaw = sin(0.01 * dt++);
         
        gimbal_angle_.yaw_angle   = yaw * 180 / M_PI;
        //gimbal_angle_.pitch_angle = 0;
        ros_ctrl_gimbal_angle_.publish(gimbal_angle_);

        ros::spinOnce();
        loop_rate.sleep();
      }
    }
}


void AI_Test::start()
{
    ROS_ERROR("ai thrad start.");
    execute_thread_ = std::thread(&AI_Test::ExecuteLoop, this);

    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
}

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "armor_detection_node_test_client");
  roborts_decision::AI_Test ac;
  ac.start();

  return 0;
}
