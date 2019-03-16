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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>

#include "gimbal_control.h"

namespace roborts_detection {

void GimbalContrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
  offset_.x = x;
  offset_.y = y;
  offset_.z = z;
  offset_pitch_ = pitch;
  offset_yaw_ = yaw;
  init_v_ = init_v;
  init_k_ = init_k;
}

void GimbalContrl::SolveContrlAgnle(cv::Point3f &postion, float &yaw, float &pitch) 
{
    double down_t = postion.z / 100.0 / 15; // bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    double xyz[3] = {postion.x, postion.y - offset_gravity, postion.z};
    double alpha = 0.0, theta = 0.0;

    alpha = asin(1/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    if(xyz[1] < 0){
        theta = atan(-xyz[1]/xyz[2]);
        pitch = -(alpha+theta);  // camera coordinate
    }
    else if (xyz[1] < 1){
        theta = atan(xyz[1]/xyz[2]);
        pitch = -(alpha-theta);  // camera coordinate
    }
    else{
        theta = atan(xyz[1]/xyz[2]);
        pitch = (theta-alpha);   // camera coordinate
    }
    yaw = atan2(xyz[0], xyz[2]);
    yaw = yaw * 180 / 3.1415926;
    pitch = pitch * 180 / 3.1415926;
}

} // roborts_detection



