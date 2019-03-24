
#include <unistd.h>
#include "apriltags_detection_node.h"

namespace roborts_detection {

TagsDetectionNode::TagsDetectionNode():
    node_state_(roborts_common::IDLE),
    initialized_(false),
    detected_tag_(false),
    as_(nh_, "tag_detection_node_action", boost::bind(&TagsDetectionNode::ActionCB, this, _1), false) 
{
    if (Init().IsOK()) {
      initialized_ = true;
      node_state_ = roborts_common::IDLE;
    } else {
      ROS_ERROR("apriltags_detection_node initalized failed!");
      node_state_ = roborts_common::FAILURE;
    }
    as_.start();
}

ErrorInfo TagsDetectionNode::Init() 
{
  return ErrorInfo(ErrorCode::OK);
}

void TagsDetectionNode::ActionCB(const roborts_msgs::TagDetectionGoal::ConstPtr &data) {
  roborts_msgs::TagDetectionFeedback feedback;
  roborts_msgs::TagDetectionResult result;
  bool undetected_msg_published = false;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }
  ros::Rate rate(100);
  while(ros::ok()) {

    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }
    
    {
      std::lock_guard<std::mutex> guard(mutex_);

      feedback.enemy_pos.clear();
      feedback.error_code = error_info_.error_code();
      feedback.error_msg = error_info_.error_msg();

      if (detected_tag_){
        feedback.detected = true;
        for (int i = 0; i != targets_3d_.size(); ++i)
        {
          geometry_msgs::Point temp;
          temp.x = targets_3d_[i].x;
          temp.y = targets_3d_[i].y;
          temp.z = targets_3d_[i].z;
          feedback.enemy_pos.push_back(temp);
        }
      }else{
        feedback.detected = false;
      }
      as_.publishFeedback(feedback);
    }
    rate.sleep();
  }
}

void TagsDetectionNode::ExecuteLoop() 
{
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) 
    {
      std::vector<cv::Point3f> targets_3d;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_tag_, targets_3d);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        targets_3d_ = targets_3d;
        error_info_ = error_info;
      }
    } 
    else if (node_state_ == NodeState::PAUSE)    // 如果暂停
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);                 // stop here
    }
  }
}

void TagsDetectionNode::StartThread() 
{
  ROS_INFO("Armor detection node started!");
  running_ = true;
  armor_detector_->SetThreadState(true);
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&TagsDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void TagsDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void TagsDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

TagsDetectionNode::~TagsDetectionNode() {
  StopThread();
}
} //namespace roborts_detection

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "apriltags_detection_node", ros::init_options::NoSigintHandler);
  roborts_detection::TagsDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}
