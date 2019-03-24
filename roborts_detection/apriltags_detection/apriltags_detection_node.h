
#ifndef ROBORTS_DETECTION_APRILTAGS_DETECTION_NODE_H
#define ROBORTS_DETECTION_APRILTAGS_DETECTION_NODE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/TagDetectionAction.h"

#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "state/node_state.h"

namespace roborts_detection {

using roborts_common::NodeState;
using roborts_common::ErrorInfo;

class TagsDetectionNode {
 public:
  explicit TagsDetectionNode();
  /**
   * @brief Initializing armor detection algorithm.
   * @return Return the error information.
   */
  ErrorInfo Init();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void ActionCB(const roborts_msgs::TagDetectionGoal::ConstPtr &data);
  /**
   * @brief Starting the armor detection thread.
   */
  void StartThread();
  /**
   * @brief Pausing the armor detection thread when received command 2 in action_lib callback function.
   */
  void PauseThread();
  /**
   * @brief Stopping armor detection thread.
   */
  void StopThread();
  /**
   * @brief Executing the armor detection algorithm.
   */
  void ExecuteLoop();

  ~TagsDetectionNode();
 protected:
 private:
  std::shared_ptr<TagDetectionBase> armor_detector_;
  std::thread tag_detection_thread_;

  //! state and error
  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;

  //! enemy information
  bool detected_tag_;

  //ROS
  ros::NodeHandle nh_;
  std::shared_ptr<CVToolbox> cv_toolbox_;
  actionlib::SimpleActionServer<roborts_msgs::TagDetectionAction> as_;

  std::vector<cv::Point3f> targets_3d_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_APRILTAGS_DETECTION_NODE_H
