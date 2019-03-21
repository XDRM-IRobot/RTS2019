#include "ai_test.h"

namespace roborts_decision{

void AI_Test::NavGoalCallback(const geometry_msgs::PoseStamped & goal)
  {
    ROS_INFO("Get nav goal.");
    global_planner_goal_.goal = goal;
    global_planner_actionlib_client_.sendGoal(global_planner_goal_,
                                              boost::bind(&AI_Test::DoneCallback, this, _1, _2),
                                              boost::bind(&AI_Test::ActiveCallback, this),
                                              boost::bind(&AI_Test::FeedbackCallback, this, _1)
    );
  }

  
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
}