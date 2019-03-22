
#include "ai_test.h"

namespace roborts_decision{
    
void AI_Test::GetEnemyGloalPose(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback)
  {
      ptz_point_candidate_.clear();
      global_pose_candidate_.clear();

      for (int i = 0; i != feedback->enemy_pos.size(); ++i)
      {
        // gimbal
        geometry_msgs::Point pt;
        pt.x = feedback->enemy_pos[i].x - gimbal_control_.offset_.x;
        pt.y = feedback->enemy_pos[i].y - gimbal_control_.offset_.y;
        pt.z = feedback->enemy_pos[i].z - gimbal_control_.offset_.z;
        ptz_point_candidate_.push_back(pt);

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

        float distance = ptz_pose.pose.position.x;
        if (distance > 2)
        {
          geometry_msgs::PoseStamped nav_pose = ptz_pose;
          nav_pose.pose.position.x -= 2; 
          //tf::Transform transform
          try{
            geometry_msgs::PoseStamped ptz_pose_in_map, 
                                       nav_pose_in_map;
            tf::Stamped<tf::Pose> ptz_pose_tf, 
                                  nav_pose_tf;

            tf_ptr_->transformPose("map", ptz_pose, ptz_pose_in_map);
            tf_ptr_->transformPose("map", nav_pose, nav_pose_in_map);

            poseStampedMsgToTF(ptz_pose_in_map, ptz_pose_tf);
            poseStampedMsgToTF(nav_pose_in_map, nav_pose_tf);

            tf_in_map_.sendTransform(tf::StampedTransform(ptz_pose_tf, ros::Time::now(), "map", "enemy_link"));
            tf_in_map_.sendTransform(tf::StampedTransform(nav_pose_tf, ros::Time::now(), "map", "nav_link"));
            //global_pose_candidate_.push_back(map_pose);

            NavGoalCallback(nav_pose);
          }
          catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("tf error when transform enemy pose from /gimbal_link to /map");
            ros::Duration(1.0).sleep();
          }
        }
      }
  }
  
  unsigned long dt = 0;
 

  void AI_Test::ArmorDetectionCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected)
    {
        enemy_detected_ = true;
        lost_cnt_ = 0;
        ROS_INFO("Find Enemy!");
        GetEnemyGloalPose(feedback);
        SelectFinalEnemy();
    } 
    else
      {
        enemy_detected_ = false;
        lost_cnt_ = 0;
        ROS_ERROR("no armor");
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
