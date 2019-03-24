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

#include <ros/ros.h>
#include <ros/package.h>

#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"


class BaseTest
{
public:
  BaseTest()
  {
    ros_nh_                      = ros::NodeHandle();
    //ros_game_status_sub_     = ros_nh_.subscribe("game_status", 1, &BaseTest::game_status_Callback, this);
    //ros_game_result_sub_     = ros_nh_.subscribe("game_result", 1, &BaseTest::game_result_Callback, this);
    //ros_game_survival_sub_   = ros_nh_.subscribe("game_survivor", 1, &BaseTest::game_survivor_Callback, this);
    //ros_bonus_status_sub_    = ros_nh_.subscribe("field_bonus_status", 1, &BaseTest::field_bonus_status_Callback, this);
    //ros_supplier_status_sub_ = ros_nh_.subscribe("field_supplier_status", 1, &BaseTest::field_supplier_status_Callback, this);
    ros_robot_status_sub_    = ros_nh_.subscribe("robot_status", 1, &BaseTest::robot_status_Callback, this);
    //ros_robot_heat_sub_      = ros_nh_.subscribe("robot_heat", 1, &BaseTest::robot_heat_Callback, this);
    //ros_robot_bonus_sub_     = ros_nh_.subscribe("robot_bonus", 1, &BaseTest::robot_bonus_Callback, this);
    ros_robot_damage_sub_    = ros_nh_.subscribe("robot_damage", 1, &BaseTest::robot_damage_Callback, this);
    //ros_robot_shoot_sub_     = ros_nh_.subscribe("robot_shoot", 1, &BaseTest::robot_shoot_Callback, this);

  }
  void game_status_Callback(const roborts_msgs::GameStatus::ConstPtr msg)
  {
    game_status_.game_status    = msg->game_status;
    game_status_.remaining_time = msg->remaining_time;
    ROS_ERROR("game_status_.game_status : %d", game_status_.game_status);
    ROS_ERROR("game_status_.remaining_time : %d", game_status_.remaining_time);
  }
  void game_result_Callback(const roborts_msgs::GameResult::ConstPtr msg)
  {
    game_result_.result = msg->result;
    ROS_ERROR("game_result_.result : %d", game_result_.result);
  }
  void game_survivor_Callback(const roborts_msgs::GameSurvivor::ConstPtr msg)
  {
    game_survivor_.blue3 = msg->blue3;
    game_survivor_.blue4 = msg->blue4;
    game_survivor_.red3  = msg->red3;
    game_survivor_.red4  = msg->red4;
    ROS_ERROR("game_survivor_.blue3 : %d", game_survivor_.blue3);
    ROS_ERROR("game_survivor_.blue4 : %d", game_survivor_.blue4);
    ROS_ERROR("game_survivor_.red3 : %d", game_survivor_.red3);
    ROS_ERROR("game_survivor_.red4 : %d", game_survivor_.red4);
  }

  void field_bonus_status_Callback(const roborts_msgs::BonusStatus::ConstPtr msg)
  {
    bonus_status_.red_bonus = msg->red_bonus;
    bonus_status_.blue_bonus = msg->blue_bonus;
    ROS_ERROR("bonus_status_.red_bonus : %d", bonus_status_.red_bonus);
    ROS_ERROR("bonus_status_.blue_bonus : %d", bonus_status_.blue_bonus);
  }
  void field_supplier_status_Callback(const roborts_msgs::SupplierStatus::ConstPtr msg)
  {
    supplier_status_.status = msg->status;
    ROS_ERROR("supplier_status_.status : %d", supplier_status_.status);
  }
  void robot_status_Callback(const roborts_msgs::RobotStatus::ConstPtr msg)
  {

    robot_status_.id                 = msg->id;
    robot_status_.level              = msg->level;
    robot_status_.remain_hp          = msg->remain_hp;
    robot_status_.max_hp             = msg->max_hp;
    robot_status_.heat_cooling_limit = msg->heat_cooling_limit;
    robot_status_.heat_cooling_rate  = msg->heat_cooling_rate;
    robot_status_.gimbal_output      = msg->gimbal_output;
    robot_status_.chassis_output     = msg->chassis_output;
    robot_status_.shooter_output     = msg->shooter_output;
    //ROS_ERROR("robot_status_.id : %d", robot_status_.id);
    //ROS_ERROR("robot_status_.remain_hp : %d", robot_status_.remain_hp);
  }

  void robot_heat_Callback(const roborts_msgs::RobotHeat::ConstPtr msg)
  {
    robot_heat_.chassis_volt         = msg->chassis_volt;
    robot_heat_.chassis_current      = msg->chassis_current;
    robot_heat_.chassis_power        = msg->chassis_power;
    robot_heat_.chassis_power_buffer = msg->chassis_power_buffer;
    robot_heat_.shooter_heat         = msg->shooter_heat;
  }

  void robot_bonus_Callback(const roborts_msgs::RobotBonus::ConstPtr msg)
  {
    robot_bonus_.bonus = msg->bonus;
  }
  void robot_damage_Callback(const roborts_msgs::RobotDamage::ConstPtr msg)
  {
    robot_damage_.damage_source = msg->damage_source;
    ROS_ERROR("robot_damage_.damage_source : %d", robot_damage_.damage_source);
  }
  void robot_shoot_Callback(const roborts_msgs::RobotShoot::ConstPtr msg)
  {
    robot_shoot_.frequency = msg->frequency;
    robot_shoot_.speed     = msg->speed;
  }

private:
  //! ros control
  ros::NodeHandle    ros_nh_;
  ros::Subscriber ros_game_status_sub_;
  ros::Subscriber ros_game_result_sub_;
  ros::Subscriber ros_game_survival_sub_;

  ros::Subscriber ros_bonus_status_sub_;
  ros::Subscriber ros_supplier_status_sub_;

  ros::Subscriber ros_robot_status_sub_ ;
  ros::Subscriber ros_robot_heat_sub_;
  ros::Subscriber ros_robot_bonus_sub_;
  ros::Subscriber ros_robot_damage_sub_;
  ros::Subscriber ros_robot_shoot_sub_;

  roborts_msgs::BonusStatus      bonus_status_;
  roborts_msgs::GameResult       game_result_;
  roborts_msgs::GameStatus       game_status_;
  roborts_msgs::GameSurvivor     game_survivor_;
  roborts_msgs::ProjectileSupply projectile_supply_;
  roborts_msgs::RobotBonus       robot_bonus_;
  roborts_msgs::RobotDamage      robot_damage_;
  roborts_msgs::RobotHeat        robot_heat_;
  roborts_msgs::RobotShoot       robot_shoot_;
  roborts_msgs::RobotStatus      robot_status_;
  roborts_msgs::SupplierStatus   supplier_status_;
};


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "armor_detection_node_test_client");
  ROS_ERROR("listen to referee.");
  BaseTest test;
  ros::MultiThreadedSpinner spinner(4);
  ros::spin();
  
  return 0;
}
