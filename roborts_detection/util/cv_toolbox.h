/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify *  it under the terms of the GNU General Public License as published by *  the Free Software Foundation, either version 3 of the License, or *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DETECTION_CVTOOLBOX_H
#define ROBORTS_DETECTION_CVTOOLBOX_H

#include <vector>
#include <thread>
#include <mutex>
//opencv
#include <opencv2/opencv.hpp>
//ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace roborts_detection {

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

/**
 * This class is a toolbox for RoboMaster detection.
 */

enum BufferState {
  IDLE = 0,
  WRITE,
  READ
};

class CVToolbox {
 public:
  /**
   *  @brief The constructor of the CVToolbox.
   */
  explicit CVToolbox(std::string camera_name,
                     unsigned int buffer_size = 3) :
      get_img_info_(false)
  {

    ros::NodeHandle nh(camera_name);
    image_transport::ImageTransport it(nh);

    camera_sub_ = it.subscribeCamera("image_raw",
                                     20,
                                     boost::bind(&CVToolbox::ImageCallback, this, _1,_2));

    image_buffer_.resize(buffer_size);
    buffer_state_.resize(buffer_size);
    index_ = 0;
    capture_time_ = -1;
    for (int i = 0; i < buffer_state_.size(); ++i) {
      buffer_state_[i] = BufferState::IDLE;
    }
    latest_index_ = -1;

  }

  int GetCameraHeight() {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera height info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.height;
  }

  int GetCameraWidth() {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera width info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.width;
  }

  int GetCameraMatrix(cv::Mat &k_matrix) {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera matrix info, because the first frame data wasn't received");
      return -1;
    }
    k_matrix = cv::Mat(3, 3, CV_64F, camera_info_.K.data()).clone();
    return 0;
  }

  int GetCameraDistortion(cv::Mat &distortion) {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera distortion info, because the first frame data wasn't received");
      return -1;
    }
    distortion = cv::Mat(camera_info_.D.size(), 1, CV_64F, camera_info_.D.data()).clone();
    return 0;
  }

  int GetWidthOffSet() {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera width offset info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.roi.x_offset;
  }

  int GetHeightOffSet() {
    if (!get_img_info_) {
      ROS_WARN("Can not get camera height offset info, because the first frame data wasn't received");
      return -1;
    }
    return camera_info_.roi.y_offset;
  }

  void ImageCallback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
    if(!get_img_info_){
      camera_info_ = *camera_info_msg;
      capture_begin_ = std::chrono::high_resolution_clock::now();
      get_img_info_ = true;
    } else {
      capture_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>(std::chrono::high_resolution_clock::now() - capture_begin_).count();
      capture_begin_ = std::chrono::high_resolution_clock::now();
//      ROS_WARN("capture time: %lf", capture_time_);
    }
    capture_begin_ = std::chrono::high_resolution_clock::now();
    auto write_begin = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < buffer_state_.size(); ++i) {
      if (buffer_state_[i] != BufferState::READ) {
        image_buffer_[i] = cv_bridge::toCvShare(img_msg, "bgr8")->image.clone();
        buffer_state_[i] = BufferState::WRITE;
        lock_.lock();
        latest_index_ = i;
        lock_.unlock();
      }
    }
    /*
    ROS_WARN("write time: %lf", std::chrono::duration<double, std::ratio<1, 1000000>>
        (std::chrono::high_resolution_clock::now() - write_begin).count());*/
  }
  /**
   * @brief Get next new image.
   * @param src_img Output image
   */
  int NextImage(cv::Mat &src_img) {
    if (latest_index_ < 0) {
      ROS_WARN("Call image when no image received");
      return -1;
    }
    int temp_index = -1;
    lock_.lock();
    if (buffer_state_[latest_index_] == BufferState::WRITE) {
      buffer_state_[latest_index_] = BufferState::READ;
    } else {
      ROS_INFO("No image is available");
      lock_.unlock();
      return temp_index;
    }
    temp_index = latest_index_;
    lock_.unlock();

    src_img = image_buffer_[temp_index];
    return temp_index;
  }

  void GetCaptureTime(double &capture_time) {
    if (!get_img_info_) {
      ROS_WARN("The first image doesn't receive");
      return;
    }
    if (capture_time_ < 0) {
      ROS_WARN("The second image doesn't receive");
      return;
    }

    capture_time = capture_time_;
  }

  /**
   * @brief Return the image after use
   * @param return_index Index gets from function 'int NextImage(cv::Mat &src_img)'
   */
  void ReadComplete(int return_index) {

    if (return_index < 0 || return_index > (buffer_state_.size() - 1)) {
      ROS_ERROR("Return index error, please check the return_index");
      return;
    }

    buffer_state_[return_index] = BufferState::IDLE;
  }

  /**
   * @brief Highlight the blue or red region of the image.
   * @param image Input image ref
   * @return Single channel image
   */
  cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv) {
    if(using_hsv) {
      cv::Mat img_hsv;
      cv::cvtColor(src_img, img_hsv, CV_BGR2HSV);
      if (color == 0) {
        cv::Mat img_hsv_blue, img_threshold_blue;
        img_hsv_blue = img_hsv.clone();
        cv::Mat blue_low(cv::Scalar(90, 150, 46));
        cv::Mat blue_higher(cv::Scalar(140, 255, 255));
        cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
        return img_threshold_blue;
      } else {
        cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
        img_hsv_red1 = img_hsv.clone();
        img_hsv_red2 = img_hsv.clone();
        cv::Mat red1_low(cv::Scalar(0, 43, 46));
        cv::Mat red1_higher(cv::Scalar(3, 255, 255));

        cv::Mat red2_low(cv::Scalar(170, 43, 46));
        cv::Mat red2_higher(cv::Scalar(180, 255, 255));
        cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
        cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
        img_threshold_red = img_threshold_red1 | img_threshold_red2;
        //cv::imshow("img_threshold_red", img_threshold_red);
        return img_threshold_red;
      }
    } else {
      std::vector<cv::Mat> bgr;
      cv::split(src_img, bgr);
      if (color == 1) {
        cv::Mat result_img;
        cv::subtract(bgr[2], bgr[1], result_img);
        return result_img;
      } else if (color == 0) {
        cv::Mat result_img;
        cv::subtract(bgr[0], bgr[2], result_img);
        return result_img;
      }
    }
  }
  /**
   * @brief The wrapper for function cv::findContours
   * @param binary binary image ref
   * @return Contours that found.
   */
  std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    const auto mode = CV_RETR_EXTERNAL;
    const auto method = CV_CHAIN_APPROX_SIMPLE;
    cv::findContours(binary_img, contours, mode, method);
    return contours;
  }
  /**
   * @brief Draw rectangle.
   * @param img The image will be drew on
   * @param rect The target rectangle
   * @param color Rectangle color
   * @param thickness Thickness of the line
   */
  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    float angle = rect.angle;
    std::ostringstream ss;
    ss << angle;
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness, float angle) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    std::ostringstream ss;
    ss << (int)(angle);
    std::string text(ss.str());
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 100, 0), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }

  void DrawTarget3d(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness, cv::Point3f pt) {
    cv::Point2f vertex[4];

    cv::Point2f center = rect.center;
    std::ostringstream ss;
    ss << pt.x << ", " << pt.y << ", " << pt.z;
    std::string text(ss.str());

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.5;
    cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }
 private:
  std::vector<cv::Mat> image_buffer_;
  std::vector<BufferState> buffer_state_;
  int latest_index_;
  std::mutex lock_;
  int index_;
  std::chrono::high_resolution_clock::time_point capture_begin_;
  double capture_time_;

  image_transport::CameraSubscriber camera_sub_;
  bool get_img_info_;
  sensor_msgs::CameraInfo camera_info_;

public:
  void adjustRect(cv:: RotatedRect &rect)
  {
    if(rect.size.width > rect.size.height)
    {
      auto temp = rect.size.height;
      rect.size.height = rect.size.width;
      rect.size.width = temp;
      rect.angle += 90;
      if(rect.angle > 180)
        rect.angle -= 180;
    }
      
    if(rect.angle > 90)
        rect.angle -= 90;
    else if(rect.angle < -90)
        rect.angle += 90;   // 左灯条角度为负, 右灯条角度为正
  }
  /**
   * @brief: 针对平凡的情况
   */
  cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
  }
  /**
   * @brief: 针对快速平移的情况
   */
  cv::RotatedRect boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.width, wh_r.width);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
  }
  /**
   * @brief: 针对慢速平移的情况
   */
  cv::RotatedRect boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const cv::Point & pl = left.center, & pr = right.center;
    cv::Point2f center; 
    center.x = (pl.x + pr.x) / 2.0;
    center.y = (pl.y + pr.y) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
  }
  bool makeRectSafe(cv::Rect & rect, cv::Size size){
      if (rect.x < 0)
          rect.x = 0;
      if (rect.x + rect.width > size.width)
          rect.width = size.width - rect.x;
      if (rect.y < 0)
          rect.y = 0;
      if (rect.y + rect.height > size.height)
          rect.height = size.height - rect.y;
      if (rect.width <= 0 || rect.height <= 0)
          return false;
      return true;
  }

  int armorToarmorTest(const cv::RotatedRect & _rect1, const cv::RotatedRect & _rect2)
  {
    cv::Point2f center1 = _rect1.center;
    cv::Point2f center2 = _rect2.center;
    cv::Rect rect1 = _rect1.boundingRect();
    cv::Rect rect2 = _rect2.boundingRect();

    if (rect1.x < center2.x && center2.x < rect1.x + rect1.width  && 
        rect2.x < center1.x && center1.x < rect2.x + rect2.width  && 
        rect1.y < center2.y && center2.y < rect1.y + rect1.height &&
        rect2.y < center1.y && center1.y < rect2.y + rect2.height )
    {
			if(_rect1.size.area() > _rect2.size.area()) return 1;
			else return 2;
		}
	  return -1;
  }
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_CVTOOLBOX_H
