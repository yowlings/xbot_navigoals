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
 * @file /kobuki_keyop/src/navigoal_core.cpp
 *
 * @brief Creates a node for remote controlling parts of robot_core.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include "../include/navigoal_core/navigoal_core.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace navigoal_core
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
NaviGoalCore::NaviGoalCore() : key_file_descriptor(0),
                               num_goals(0),
                               click_goal_finished(true),
                               quit_requested(false),
                               current_goal(0),
                               reached_goal(true)

{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

NaviGoalCore::~NaviGoalCore()
{
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool NaviGoalCore::init()
{
  ros::NodeHandle nh("~");

  name = nh.getUnresolvedNamespace();  
  /*********************
   ** Subscribers
   **********************/

  clicked_goal_subscriber = nh.subscribe("/goal",1,&NaviGoalCore::clickedGoalReceived, this);
  navigoal_status = nh.subscribe("/move_base/result", 1,&NaviGoalCore::naviGoalStatus, this);


  /*********************
   ** Publishers
   **********************/
  plans_publisher_ = nh.advertise<nav_msgs::Path>("navi_plans",1);
  execute_goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  marker_goals_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("marker_goals",0);


  /*********************
   ** Service
   **********************/
  serviceClient = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
  if (!serviceClient) {
  ROS_FATAL("Could not initialize get plan service from %s, please make sure that move_base is activated!",
  serviceClient.getService().c_str());
  }
  navi_path.poses.clear();
  navi_path.header.frame_id = "map";




  /*********************
   ** Marker_goals
   **********************/
  marker.header.frame_id = "map";
  marker.ns = "my_namespace";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 0;





  // start keyboard input thread
  thread.start(&NaviGoalCore::keyboardInputLoop, *this);
  return true;
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop; sends current velocity command at a fixed rate.
 *
 * It also process ros functions as well as aborting when requested.
 */
void NaviGoalCore::spin()
{
  ros::Rate loop_rate(10);

  while (!quit_requested&&ros::ok())
  {
    if(click_goal_finished)
    {

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  thread.join();
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void NaviGoalCore::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("n : create a new goal serial.");
  puts("d : delete last goal.");
  puts("c : clear all goals.");
  puts("f : finish click goal.");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}


/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void NaviGoalCore::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {    
    case 'n':
    {
      createGoalSerials();
      break;
    }
    case 'd':
    {
      deletLastGoal();
      break;
    }
    case 'f':
    {
      completeGoals();
      break;
    }
    case 'c':
    {
      clearAllGoals();
      break;
    }
    case 'q':
    {
      quit_requested=true;
      break;
    }
    default:
    {
      break;
    }
  }
}

// move_base/result callback function..

void NaviGoalCore::naviGoalStatus(const move_base_msgs::MoveBaseActionResult& result)
{
  current_goal=(current_goal+1>num_goals)?0:current_goal+1;
  execute_goal_publisher_.publish(goals[current_goal]);

}


//2D Nav Goal tool clicked callback function..
void NaviGoalCore::clickedGoalReceived(const geometry_msgs::PoseStamped &pose)
{
  if(!click_goal_finished)
  {
    if(num_goals<MAX_GOAL_NUM)
    {
      goals[num_goals]=pose;
      num_goals++;
      ROS_INFO("addded a new goal:\n position:\n x: %f\n y: %f\n There are %d/%d goals now!\n",pose.pose.position.x,pose.pose.position.y,num_goals,MAX_GOAL_NUM);
      marker.header.stamp = ros::Time();
      marker.id = num_goals;

      marker.pose = pose.pose;
      arraymarker.markers.push_back(marker);
      marker_goals_publisher_.publish( arraymarker );

      if(num_goals>1)
      {
        make_plan_srv.request.start = goals[num_goals-2];
        make_plan_srv.request.goal = goals[num_goals-1];
        make_plan_srv.request.tolerance = 0.5;

        if(serviceClient.call(make_plan_srv))
        {
          if(!make_plan_srv.response.plan.poses.empty())
          {
//            ROS_INFO(make_plan_srv.response.plan.header.frame_id);
            for(int i=0;i<make_plan_srv.response.plan.poses.size();i++)
            {

              navi_path.poses.push_back(make_plan_srv.response.plan.poses[i]);
            }
          }
          else
          {
            ROS_WARN("Got empty plan");
          }
        }
        else
        {
          ROS_ERROR("Can not call service!");
        }

      }

      plans_publisher_.publish(navi_path);
    }
    else
    {
      ROS_INFO("The number of goals are overflowed, the max number is %d, but there are %d/%d goals now!",MAX_GOAL_NUM, num_goals,MAX_GOAL_NUM);
    }

  }

}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/


/*** @brief create a new goal serial..
 */

void NaviGoalCore::createGoalSerials()
{
  if(click_goal_finished)
  {
    click_goal_finished=false;
    num_goals=0;
    arraymarker.markers.clear();
    navi_path.poses.clear();
    ROS_INFO("Initialized, please enter your goals.");
  }
  else
  {
    ROS_INFO("You are clicking a goal serial, want to clear them?, please press 'c' key!");
  }


}

/*** clear all goals..
 */
void NaviGoalCore::clearAllGoals()
{
  if(!click_goal_finished)
  {
    num_goals=0;
    arraymarker.markers.clear();
    ROS_INFO_STREAM("All of the goals are cleared");
  }

}

/**
 * @brief delete last fault goal..
 */
void NaviGoalCore::deletLastGoal()
{
  if(!click_goal_finished){
    if(num_goals>0)
    {
      num_goals--;
      arraymarker.markers.pop_back();
      ROS_INFO("Last goal is deleted, there are %d/%d goals now!", num_goals, MAX_GOAL_NUM);
    }
    else
    {
      ROS_ERROR_STREAM("You have not given any goals!");
    }
  }


}

/**
 * @brief click goal complete..
 */
void NaviGoalCore::completeGoals()
{
  make_plan_srv.request.start = goals[num_goals-1];
  make_plan_srv.request.goal = goals[0];
  make_plan_srv.request.tolerance = 0.5;

  if(serviceClient.call(make_plan_srv))
  {
    if(!make_plan_srv.response.plan.poses.empty())
    {
      for(int i=0;i<make_plan_srv.response.plan.poses.size();i++)
      {
        navi_path.poses.push_back(make_plan_srv.response.plan.poses[i]);
      }
    }
    else
    {
      ROS_WARN("Got empty plan");
    }
  }
  plans_publisher_.publish(navi_path);
  click_goal_finished = true;
  reached_goal = false;
  execute_goal_publisher_.publish(goals[0]);

  ROS_INFO_STREAM("Click goals finished!");

}


} // namespace navigoal_core
