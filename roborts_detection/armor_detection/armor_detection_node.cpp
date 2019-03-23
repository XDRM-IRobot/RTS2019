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

#include <unistd.h>
#include "armor_detection_node.h"

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    initialized_(false),
    detected_enemy_(false),
    as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) 
  {
    if (Init().IsOK()) {
      initialized_ = true;
      node_state_ = roborts_common::IDLE;
    } else {
      ROS_ERROR("armor_detection_node initalized failed!");
      node_state_ = roborts_common::FAILURE;
    }
    as_.start();
}

ErrorInfo ArmorDetectionNode::Init() 
{
  ArmorDetectionAlgorithms armor_detection_param;

  std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  if (!read_state) {
    ROS_ERROR("Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  //create the selected algorithms
  std::string selected_algorithm = armor_detection_param.selected_algorithm();
  // create image receiver
  cv_toolbox_ = std::make_shared<CVToolbox>(armor_detection_param.camera_name());
  // create armor detection algorithm
  armor_detector_ = roborts_common::AlgorithmFactory<ArmorDetectionBase,std::shared_ptr<CVToolbox>>::CreateAlgorithm
      (selected_algorithm, cv_toolbox_);

  if (armor_detector_ == nullptr) {
    ROS_ERROR("Create armor_detector_ pointer failed!");
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}

void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) {
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
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

      if (detected_enemy_){
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

void ArmorDetectionNode::ExecuteLoop() 
{
  while(running_) {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) 
    {
      std::vector<cv::Point3f> targets_3d;
      ErrorInfo error_info = armor_detector_->DetectArmor(detected_enemy_, targets_3d);
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

void ArmorDetectionNode::StartThread() 
{
  ROS_INFO("Armor detection node started!");
  running_ = true;
  armor_detector_->SetThreadState(true);
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  armor_detector_->SetThreadState(false);
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
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
  ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}
