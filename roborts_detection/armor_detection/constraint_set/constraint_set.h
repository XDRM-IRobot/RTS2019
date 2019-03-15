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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H

//system include
#include <vector>
#include <list>

#include <opencv2/opencv.hpp>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"

#include "cv_toolbox.h"

#include "../armor_detection_base.h"

#include "proto/constraint_set.pb.h"
#include "constraint_set.h"
namespace roborts_detection {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

enum State {
  INITIALIZED = 0,
  RUNNING = 1,
  FAILED = 2,
  STOPED = 3
};

struct LightInfo {

  explicit LightInfo(cv::Point2f vertices[]) {
    auto edge_1 = std::pow(vertices[0].x - vertices[1].x, 2) +
        std::pow(vertices[0].y - vertices[1].y, 2);
    auto edge_2 = std::pow(vertices[1].x - vertices[2].x, 2) +
        std::pow(vertices[1].y - vertices[2].y, 2);

    if (edge_1 > edge_2) {
      width_  = (float)std::sqrt(edge_1);
      height_ = (float)std::sqrt(edge_2);

      if (vertices[0].y < vertices[1].y) {
        angle_ = std::atan2(vertices[1].y - vertices[0].y, vertices[1].x - vertices[0].x);
      } else {
        angle_ = std::atan2(vertices[0].y - vertices[1].y, vertices[0].x - vertices[1].x);
      }

    } else {
      width_  = (float)std::sqrt(edge_2);
      height_ = (float)std::sqrt(edge_1);

      if (vertices[2].y < vertices[1].y) {
        angle_ = std::atan2(vertices[1].y - vertices[2].y, vertices[1].x - vertices[2].x);
      } else {
        angle_ = std::atan2(vertices[2].y - vertices[1].y, vertices[2].x - vertices[1].x);
      }

    }

    angle_ = (float)(angle_*180.0 / 3.1415926);
    area_ = width_ * height_;
    aspect_ratio_ = width_ / height_;
    center_.x = (vertices[1].x + vertices[3].x) / 2;
    center_.y = (vertices[1].y + vertices[3].y) / 2;
    vertices_.push_back(vertices[0]);
    vertices_.push_back(vertices[1]);
    vertices_.push_back(vertices[2]);
    vertices_.push_back(vertices[3]);
  }

 public:
  //! Light area
  float area_;
  //! Light angle, come from the long edge's slope
  float angle_;
  //! Light center
  cv::Point2f center_;
  //! Light aspect ratio = width_/height_
  float aspect_ratio_;
  //! Light width
  float width_;
  //! Light height
  float height_;
  //! Light vertices
  std::vector<cv::Point2f> vertices_;
};

enum Armor_Twist { STILL = 1, LOW_MOVE = 2, MID_MOVE = 3, FAST_MOVE = 4 }; // 速度信息

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
  ArmorInfo(cv::RotatedRect armor_rect, Armor_Twist st) {
    rect = armor_rect;
    state = st;
  }
 public:
  cv::RotatedRect rect;
  Armor_Twist state;
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ConstraintSet : public ArmorDetectionBase {
 public:
  ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox);
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam() override;
  /**
   * @brief The entrance function of armor detection.
   * @param translation Translation information of the armor relative to the camera.
   * @param rotation Rotation information of the armor relative to the camera.
   */
  ErrorInfo DetectArmor(bool &detected, std::vector<cv::Point3f> &targets_3d) override;
  /**
   * @brief Detecting lights on the armors.
   * @param src Input image
   * @param lights Output lights information
   */
  void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);
  /**
   * @brief Filtering the detected lights.
   * @param lights Filtered lights
   */
  void FilterLights(std::vector<cv::RotatedRect> &lights);
  /**
   * @brief Finding possible armors.
   * @param lights Take lights information as input.
   * @param armors Possible armors
   */
  void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
  /**
   * @brief Filtering Detected armors by standard deviation and non-maximum suppression(nms).
   * @param armors Result armors
   */
  void FilterArmors(std::vector<ArmorInfo> &armors);
  /**
   * @brief Slecting final armor as the target armor which we will be shot.
   * @param Input armors
   */
  ArmorInfo SlectFinalArmor(std::vector<ArmorInfo> &armors);
  /**
   *
   * @param armor
   * @param distance
   * @param pitch
   * @param yaw
   * @param bullet_speed
   */
  void CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d);

  /**
   * @brief Using two lights(left light and right light) to calculate four points of armor.
   * @param armor_points Out put
   * @param left_light Rotated rect of left light
   * @param right_light Rotated rectangles of right light
   */
  void CalcArmorInfo(std::vector<cv::Point2f> &armor_points, cv::RotatedRect left_light, cv::RotatedRect right_light);
  /**
   * @brief Calculating the coordinates of the armor by its width and height.
   * @param width Armor width
   * @param height Armor height
   */
  void SolveArmorCoordinate(const float width, const float height);
  /**
   *
   */
  void SignalFilter(double &new_num, double &old_num,unsigned int &filter_count, double max_diff);

  void SetThreadState(bool thread_state) override;
  /**
   * @brief Destructor
   */
  ~ConstraintSet() final;
 private:
  ErrorInfo error_info_;
  unsigned int filter_x_count_;
  unsigned int filter_y_count_;
  unsigned int filter_z_count_;
  unsigned int filter_distance_count_;
  unsigned int filter_pitch_count_;
  unsigned int filter_yaw_count_;

  cv::Mat src_img_;
  cv::Mat gray_img_;
  //! Read image index
  int read_index_;
  //! detection time
  double detection_time_;

  // Parameters come form .prototxt file
  bool enable_debug_;
  bool using_hsv_;
  unsigned int enemy_color_;

  //! Use for debug
  cv::Mat show_lights_before_filter_;
  cv::Mat show_lights_after_filter_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;

  // image threshold parameters
	float light_threshold_;
	float color_threshold_;
  float blue_threshold_;
  float red_threshold_;

  // light threshold parameters
  float light_min_area_;
	float light_max_area_;
	float light_min_angle_;
  float light_max_angle_;
	float light_min_angle_diff_;
	float light_max_angle_diff_;
	float light_min_aspect_ratio_;
	float light_max_aspect_ratio_;

	// armor threshold parameters
	float light_max_width_diff_;
	float light_max_height_diff_;
  float armor_min_area_;
	float armor_max_area_;
	float armor_min_angle_;
  float armor_max_angle_;
  float armor_light_angle_diff_;
	float armor_min_ratio_;
	float armor_max_ratio_;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;
	float filter_armor_area_;

  bool thread_running_;

  //ros
  ros::NodeHandle nh;

  // solver
  double target_width_;
  double target_height_;

  //! target 2d coordinates
  std::vector<cv::Point2f> target_points_2d_;
  //! target 3d coordinates
  std::vector<cv::Point3f> target_points_3d_;
  //! Camera intrinsic matrix
  cv::Mat intrinsic_matrix_;
  //! Camera distortion Coefficient
  cv::Mat distortion_coeffs_;

  void GetTarget2d(const cv::RotatedRect & rect, const cv::Point2f& offset = cv::Point2f(0,0))
  {
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y) {
        lu = vertices[0];
        ld = vertices[1];
    }
    else{
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
        ru = vertices[2];
        rd = vertices[3];
    }
    else {
        ru = vertices[3];
        rd = vertices[2];
    }

    target_points_2d_.clear();
    target_points_2d_.emplace_back(lu + offset);
    target_points_2d_.emplace_back(ru + offset);
    target_points_2d_.emplace_back(rd + offset);
    target_points_2d_.emplace_back(ld + offset);
  }

  void GetTarget3d(const cv::RotatedRect & rect, cv::Point3f & target_3d) 
  {
    if (rect.size.height < 1) return;
    double wh_ratio = target_width_ / target_height_;
    cv::RotatedRect rect_adjust(rect.center, cv::Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
    GetTarget2d(rect_adjust);

    cv::Mat rvec;
    cv::Mat tvec;
    cv::solvePnP(target_points_3d_, 
                 target_points_2d_, 
                 intrinsic_matrix_, 
                 distortion_coeffs_, 
                 rvec, 
                 tvec);
    target_3d = cv::Point3f(tvec);
  }
};

roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase, "constraint_set", ConstraintSet, std::shared_ptr<CVToolbox>);

} //namespace roborts_detection

#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
