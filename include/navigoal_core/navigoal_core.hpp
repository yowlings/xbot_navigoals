/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/teleop_core/keyop_core.hpp
 *
 * @brief The controlling node for remote operations on robot_core.
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef NAVIGOAL_CORE_NODE_HPP_
#define NAVIGOAL_CORE_NODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseActionResult.h>




#define MAX_GOAL_NUM 10
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace navigoal_core
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class NaviGoalCore
{
public:
  /*********************
   ** C&D
   **********************/
  NaviGoalCore();
  ~NaviGoalCore();
  bool init();

  /*********************
   ** Runtime
   **********************/
  void spin();

private:

  ros::Subscriber clicked_goal_subscriber;
  ros::Subscriber navigoal_status;

  ros::Publisher plans_publisher_;
  ros::Publisher execute_goal_publisher_;
  ros::Publisher marker_goals_publisher_;



  visualization_msgs::MarkerArray arraymarker;
  visualization_msgs::Marker marker;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped goals[MAX_GOAL_NUM];


  ros::ServiceClient serviceClient;
  nav_msgs::GetPlan make_plan_srv;
  nav_msgs::Path navi_path;
  int num_goals;
  int current_goal;
  bool click_goal_finished;
  bool reached_goal;
  std::string name;

  /*********************
   ** Commands
   **********************/

  void clearAllGoals();
  void createGoalSerials();
  void completeGoals();
  void deletLastGoal();

  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void clickedGoalReceived(const geometry_msgs::PoseStamped& pose);
  void naviGoalStatus(const move_base_msgs::MoveBaseActionResult& result);
  void restoreTerminal();  
  int key_file_descriptor;
  bool quit_requested;
  struct termios original_terminal_state;
  ecl::Thread thread;
};

} // namespace keyop_core

#endif /* KEYOP_CORE_NODE_HPP_ */
