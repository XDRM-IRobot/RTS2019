/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _ARMOR_DETECT_H_
#define _ARMOR_DETECT_H_

#include <iostream>
#include <chrono>
#include <ctype.h>
#include <opencv2/ml.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "armor_info.h"

using namespace cv;
using namespace cv::ml;

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

namespace detect_mul
{
enum EnemyColor { RED = 0, BLUE = 1};

class ArmorDetector
{
public:
    ArmorDetector() {
		// svm_big = StatModel::load<SVM>("../config/big_armor_model.yml");
		// svm_small = StatModel::load<SVM>("../config/armor_model.yml");
	};
	
	/**
 	 * @brief: 检测API
	 * @param: camera_src 
	 * @param: vector<armor_info> candidate
	 * @param: Armor_recorder
 	 */
	bool detect(cv::Mat & src, std::vector<armor_info> & armors_candidate);

private:
	
private:
    armor_param _para;	// 装甲板的参数
	cv::Mat src_img_;
	cv::Mat gray_img_;
	cv::Mat show_lights_before_filter_;
    cv::Mat show_lights_after_filter_;
    cv::Mat show_armors_befor_filter_;
    cv::Mat show_armors_after_filter_;
	std::chrono::steady_clock::time_point speed_test_start_begin_time;
	
	bool last_detect;
	
	std::vector<cv::RotatedRect> light_rects;
	std::vector<armor_info> filte_rects;
	
	// 尝试过对1号步兵和2号英雄进行Hist的匹配分类
	cv::Mat armor_1_hist;  // 1号步兵的 Hist
	cv::Mat armor_2_hist;  // 2号英雄的 Hist

	// SVM效果会好很多，将来可以按照数字贴纸更细化的分类
	Ptr<SVM> svm_big;
	Ptr<SVM> svm_small;
};

} // namespace detect_mul

#endif
