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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"

#include "timer/timer.h"
#include "io/io.h"

namespace roborts_detection {

// 方便输出一些调试信息
#define LINE(str) // std::cout << "Code Line:" << __LINE__ << "\t" << str << std::endl;
#define LINE_INFO(str,str2) //std::cout << "Code Line:" << __LINE__ << "\t" << str <<":\t"<< str2 << std::endl;

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox):
    ArmorDetectionBase(cv_toolbox){
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;

  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);
}

void ConstraintSet::LoadParam() {
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  // algorithm info
  enable_debug_           = constraint_set_config_.enable_debug();
  enemy_color_            = constraint_set_config_.enemy_color();
  using_hsv_              = constraint_set_config_.using_hsv();

	// image threshold parameters
	light_threshold_        = constraint_set_config_.threshold().light_threshold();
  blue_threshold_         = constraint_set_config_.threshold().blue_threshold();
  red_threshold_          = constraint_set_config_.threshold().red_threshold();

  // light threshold parameters
  light_min_area_         = constraint_set_config_.threshold().light_min_area();
	light_max_area_         = constraint_set_config_.threshold().light_max_area();
	light_min_angle_        = constraint_set_config_.threshold().light_min_angle();
  light_max_angle_        = constraint_set_config_.threshold().light_max_angle();
	light_min_angle_diff_   = constraint_set_config_.threshold().light_min_angle_diff();
	light_max_angle_diff_   = constraint_set_config_.threshold().light_max_angle_diff();
	light_min_aspect_ratio_ = constraint_set_config_.threshold().light_min_aspect_ratio();
	light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();

	// armor threshold parameters
	light_max_width_diff_   = constraint_set_config_.threshold().light_max_width_diff();
	light_max_height_diff_  = constraint_set_config_.threshold().light_max_height_diff();
  armor_min_area_         = constraint_set_config_.threshold().armor_min_area();
	armor_max_area_         = constraint_set_config_.threshold().armor_max_area();
	armor_min_angle_        = constraint_set_config_.threshold().armor_min_angle();
  armor_max_angle_        = constraint_set_config_.threshold().armor_max_angle();
  armor_light_angle_diff_ = constraint_set_config_.threshold().armor_light_angle_diff();
	armor_min_ratio_        = constraint_set_config_.threshold().armor_min_ratio();
	armor_max_ratio_        = constraint_set_config_.threshold().armor_max_ratio();
	armor_min_aspect_ratio_ = constraint_set_config_.threshold().armor_min_aspect_ratio();
	armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
	filter_armor_area_      = constraint_set_config_.threshold().filter_armor_area();
	
	// angle solver parameters
  float armor_width       = constraint_set_config_.armor_size().width();
  float armor_height      = constraint_set_config_.armor_size().height();
	SolveArmorCoordinate(armor_width, armor_height);

  int get_intrinsic_state = -1;
  int get_distortion_state = -1;

  while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
    usleep(50000);
    ros::spinOnce();
    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
  }
}

ErrorInfo ConstraintSet::DetectArmor(bool &detected, std::vector<cv::Point3f> &targets_3d) 
{
  std::vector<cv::RotatedRect> lights;
  std::vector<ArmorInfo> armors;

  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) {
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
//          ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  /*ROS_WARN("time get image: %lf", std::chrono::duration<double, std::ratio<1, 1000>>
      (std::chrono::high_resolution_clock::now() - img_begin).count());*/

  auto detection_begin = std::chrono::high_resolution_clock::now();

    cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
    if (enable_debug_) {
      show_lights_before_filter_ = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_lights_after_filter_  = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_armors_befor_filter_  = src_img_.clone();
  	  show_armors_after_filter_  = src_img_.clone();
      cv::waitKey(1);
    }

    DetectLights(src_img_, lights);
    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);
		
		cv::Point3f target_3d;

    if(!armors.empty()) {
      detected = true;
			for (int i = 0; i != armors.size(); ++i)
			{
				cv::Point3f target_3d;
      	GetTarget3d(armors[i].rect, target_3d);
				targets_3d.push_back(target_3d);
				cv_toolbox_->DrawTarget3d(src_img_, armors[i].rect, cv::Scalar(0, 255, 0), 2, target_3d);
			}
    } else
      detected = false;
  
		//cv::imshow("gray_img_", gray_img_);
    cv::imshow("relust_img_", src_img_);
		cv::waitKey(1);
		
  lights.clear();
  armors.clear();
  cv_toolbox_->ReadComplete(read_index_);
  ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();

  return error_info_;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) 
{
	cv::Mat subtract_color_img;
	cv::Mat binary_brightness_img;
  cv::Mat binary_color_img;
  cv::Mat binary_light_img;

  if(using_hsv_) {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
  }else {
    cv::Mat subtract_color_img;
	  std::vector<cv::Mat> bgr_channel;
	  cv::split(src, bgr_channel);
	
	  if (enemy_color_ == RED)
		  cv::subtract(bgr_channel[2], bgr_channel[1], subtract_color_img);
	  else
		  cv::subtract(bgr_channel[0], bgr_channel[1], subtract_color_img);
		
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
    
    float thresh;
    if (enemy_color_ == RED)
      thresh = red_threshold_;
    else
      thresh = blue_threshold_;

    cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    binary_light_img = binary_color_img & binary_brightness_img;
	
  }
  std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  lights.reserve(contours_brightness.size());

  for (unsigned int i = 0; i < contours_light.size(); ++i) {
    for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      	if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) 
        {
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
          lights.push_back(single_light);
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0,255,0), 1);
          break;
        }
    } // for j loop
  } // for i loop

  if (enable_debug_)
    cv::imshow("show_lights_before_filter", show_lights_before_filter_);
}

void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) 
{
  std::vector<cv::RotatedRect> light_rects;

#pragma omp parallel for 
  for(uchar i = 0; i < lights.size(); i++ ){
  	cv_toolbox_->adjustRect(lights[i]); 	
  }
  
  for (const auto &armor_light : lights)
	{
    auto rect = std::minmax(armor_light.size.width, armor_light.size.height);
  	auto light_aspect_ratio = rect.second / rect.first;
    auto angle = armor_light.angle;

		if ( // 80 <= abs(angle) && abs(angle) <= 90   // 高速水平移动的灯条,带有拖影  // 特殊情况,无论横竖, 旧版本有这一行代码
		      light_aspect_ratio <= 2.5
		   && armor_light.size.area() > light_min_area_ // 1.0
		   && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width) // 0.04
		{
			light_rects.push_back(armor_light); // 高速水平移动的灯条
      if (enable_debug_)
		  	cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(255,0,0), 1);
		}
     // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= light_min_area_ // 1.0
		   		&& armor_light.size.area() < 100000  //light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < light_max_angle_) // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		{
			light_rects.push_back(armor_light); // 接近于垂直的灯条, 由于阈值不够合理, 细小的灯条
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,255,0), 1);
	  }
      // 检测最为平凡的情况
    else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
              armor_light.size.area() >= light_min_area_ // 1.0
			     && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
			     && abs(angle) < light_max_angle_) // 与垂直的偏角35 
    {
      light_rects.push_back(armor_light);
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,0,255), 1);
		}
  }
  if (enable_debug_)
		cv::imshow("lights_after_filter", show_lights_after_filter_);
	
  lights = light_rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armor_vector) {
    for (int i = 0; i < lights.size(); ++i) {
    for (int j = i; j < lights.size(); ++j) {
			auto rect1 = std::minmax(lights[i].size.width, lights[i].size.height);
    	auto light_aspect_ratio1 = rect1.second / rect1.first;
			auto rect2 = std::minmax(lights[j].size.width, lights[j].size.height);
    	auto light_aspect_ratio2 = rect2.second / rect2.first;

			auto angle_diff  = abs(lights[i].angle - lights[j].angle);
			auto height_diff = abs(lights[i].size.height - lights[j].size.height) / std::max(lights[i].size.height, lights[j].size.height);
			auto width_diff  = abs(lights[i].size.width - lights[j].size.width) / std::max(lights[i].size.width, lights[j].size.width);

// 快速平移的情况 Fast Move
	// 2个严格平行
			if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			    && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    && abs(lights[i].angle) == 90 && abs(lights[j].angle) == 90     // 角度为0
			    && static_cast<int>(abs(angle_diff)) % 180 == 0
			    && height_diff < 0.5 && width_diff < 0.5)	
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_ 
					&& armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个严格平行
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// 2个当中有一个略不平行
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			     	 && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    	 && ( (abs(lights[i].angle) == 90 && abs(lights[j].angle) > 80) || (abs(lights[i].angle) > 80 && abs(lights[j].angle) == 90))
			    	 && height_diff < 0.5 && width_diff < 0.5)		
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				
				// auto armor_light_angle_diff = abs(lights[i].angle) == 90 ? 
				//						 	     abs(armor_angle) + abs(armor_angle - lights[j].angle - 90); // 左右灯条的积累差

				if (armor_area > armor_min_area_
				    && armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个当中有一个略不平行 
// 快速平移的情况 Fast Move

// 中速平移的情况 Mid Move
		// 2个严格平行
			else if ( ( (light_aspect_ratio1 == 1 && light_aspect_ratio2 <= 1.5) 
					 || (light_aspect_ratio1 <= 1.5 && light_aspect_ratio2 == 1) )   // 其中一个为正方形
					 && static_cast<int>(abs(angle_diff)) % 90 == 0               	 // 角度差为0
					 && static_cast<int>(abs(lights[i].angle)) % 90 == 0          	 // 角度为0 或 90
					 && static_cast<int>(abs(lights[j].angle)) % 90 == 0          	 // 角度为0 或 90
					 && height_diff < 0.5 && width_diff < 0.5)               	     // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	    // 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			// 2个严格平行

			// 1个竖着 1个横着
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.3
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.3
					 && static_cast<int>(abs(angle_diff)) % 180 == 90           // 角度差为0
					 && ((abs(lights[i].angle) == 0 && abs(lights[j].angle) == 90) || (abs(lights[i].angle) == 90 && abs(lights[j].angle) == 0))  // 角度1个为0 1个为90
					 && height_diff < 0.5 && width_diff < 0.5)               	  // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}// 1个竖着 1个横着
// 中速平移的情况 Mid Move

// 慢速移动的情况 Low Move
		// 都是竖着的
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 && static_cast<int>(abs(angle_diff)) % 180 == 0               // 角度差为0
					 && abs(lights[i].angle) == 0 && abs(lights[j].angle) == 0     // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	 // 经验参数
					&& armor_ratio < 4                   // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )          // 经验参数
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 都是竖着的

		// 其中一块略有倾斜
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 &&	((abs(lights[i].angle) == 0  && abs(lights[j].angle) < 10) || (abs(lights[i].angle) < 10  && abs(lights[j].angle) == 0)) // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20               // 经验参数
					&& armor_light_angle_diff < 20 )
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 其中一块略有倾斜

// 慢速移动的情况 Low Move

// 平凡的情况
	// 灯条近乎平行,至少在同一侧
			else if (lights[i].angle * lights[j].angle >= 0          // 灯条近乎同侧 , 或者有一个为0
				     && abs(angle_diff) < 30                           //  light_max_angle_diff_   // 20   // 18   这些都要换成相对值
					// && height_diff < _para.light_max_height_diff      // 20  不需要宽度
					)
			{
				cv::RotatedRect possible_rect;
				// 2灯条近乎平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条近乎平行 中速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                 // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor同侧,这是中速移动的armor,1-1.5,平躺")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 中速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					// 2灯条近乎平行 快速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						
						if (armor_area > armor_min_area_
							&& armor_ratio > 1                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
						){
							Armor_Twist armor_twist = FAST_MOVE;
							LINE("[快速移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					} // 2灯条近乎平行 快速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1    // && light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 // && light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2)
						 && abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1                                         // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5                                       // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_ ) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条严格平行
			
// 灯条(误差) 并不同侧
			else if (abs(angle_diff) < light_max_angle_diff_ )     // 40
			{
				cv::RotatedRect possible_rect;
				// 2灯条 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 慢速移动
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1  // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor不同侧, 中速的armor, 竖直")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 慢速移动
				}// 2灯条 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ )// 应该要更为严格
						{
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1 //&& light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 //&& light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2))
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5 // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条(误差) 并不同侧
			else {
				cv::RotatedRect possible_rect;
				// 2灯条不平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 中速移动
					if (abs(lights[i].angle) > 60 &&  + abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			// && armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("[中速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 中速移动
				}
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[慢速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动
			}
				
		} // for j loop
	} // for i loop

  if (enable_debug_)
  {
    for (int i = 0; i != armor_vector.size(); ++i)
      cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, armor_vector[i].rect, cv::Scalar(0,255,0), 2);
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
  }
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
	std::vector<bool> is_armor(armors.size(), true);

	for (int i = 0; i < armors.size(); i++) {
	  for (int j = i + 1; j < armors.size(); j++) {
			int ojbk = cv_toolbox_->armorToarmorTest(armors[i].rect, armors[j].rect);
			if (ojbk == 1) is_armor[i] = false;
			else if(ojbk == 2) is_armor[j] = false;
		}
	}

	double dis;
#pragma omp parallel
  for (int i = 0; i < armors.size(); i++) {
#pragma omp for
    for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
    {
      dis = POINT_DIST(armors.at(i).rect.center,armors.at(j).rect.center);

      if (dis <= std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) + 
                std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) )
      {
        double armor_ratio1 = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / 
                              std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height); 
        double armor_ratio2 = std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) / 
                              std::min(armors.at(j).rect.size.width, armors.at(j).rect.size.height); 

        if (is_armor[i] == true && is_armor[j] == true)
        {
          if (abs(armor_ratio1 - armor_ratio2) > 0.4)
          {
            if(armor_ratio1 - armor_ratio2 > 0)
              is_armor[i] = false;
            else
              is_armor[j] = false;
          }
          else if (abs(abs(armors.at(i).rect.angle) - abs(armors.at(j).rect.angle)) > 15) 
          {
            if (abs(armors.at(i).rect.angle) > abs(armors.at(j).rect.angle))
              is_armor[i] = false;
            else
              is_armor[j] = false;
          }
          else if (armors.at(i).rect.size.area() - armors.at(j).rect.size.area() > 100)
          {
            is_armor[i] = false;
          }
          /*else{
            float val_i = get_armor_roi(armors[i].rect);
            float val_j = get_armor_roi(armors[j].rect);

            if(val_i > 0 && val_j < 0)
            {
              is_armor[j] = false;
            }
            else if(val_i < 0 && val_j > 0)
            {
              is_armor[i] = false;
            }
            else {
              is_armor[i] = false;
              is_armor[j] = false; 
            }*/
        }// if both
      }    // dis < dist
    }	 		 // for j
  } 			 // for i

	std::vector<ArmorInfo> filter_rects;

	for( int i = 0; i < is_armor.size();++i)
		if(is_armor[i]) filter_rects.push_back(armors[i]);
	
	armors = filter_rects;

	if (enable_debug_)
  {
    for (int i = 0; i != armors.size(); ++i)
      cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0,255,0), 2);
    cv::imshow("armors_after_filter", show_armors_after_filter_);
  }

}

ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
  std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

  return armors[0];
}

void ConstraintSet::CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d) {
  

}

void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}

void ConstraintSet::SolveArmorCoordinate(const float width, const float height) 
{
	target_width_  = width;
	target_height_ = height;
  target_points_3d_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  target_points_3d_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  target_points_3d_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  target_points_3d_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

void ConstraintSet::SetThreadState(bool thread_state) {
  thread_running_ = thread_state;
}

ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
