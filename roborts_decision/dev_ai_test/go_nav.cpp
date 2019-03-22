#include "ai_test.h"

namespace roborts_decision{

  void AI_Test::DoneCallback(const actionlib::SimpleClientGoalState& state,  
                    const roborts_msgs::GlobalPlannerResultConstPtr& result)
  {
    ROS_INFO("The goal is done with %s!",state.toString().c_str());
  }


  void AI_Test::ActiveCallback() {
    ROS_INFO("Action server has recived the goal, the goal is active!");
  }


  void AI_Test::FeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr& feedback)
  {
    if (feedback->error_code != ErrorCode::OK) {
      ROS_INFO("%s", feedback->error_msg.c_str());
    }
    if (!feedback->path.poses.empty()) {
      ROS_INFO("Get Path!");
      local_planner_goal_.route = feedback->path;                     // added
      local_planner_actionlib_client_.sendGoal(local_planner_goal_);  // added
    }
  }


  void AI_Test::GetEnemyNavGoal(geometry_msgs::PoseStamped& nav, const float distance)
  {
    nav.pose.position.x = nav.pose.position.x - distance;
    tf::Stamped<tf::Pose> nav_tf;
    poseStampedMsgToTF(nav, nav_tf);
   // tf_in_map_.sendTransform(tf::StampedTransform(nav, ros::Time::now(), "map", "nav_link"));
  }

}