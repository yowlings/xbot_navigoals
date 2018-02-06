/**
 * @file /xbot_navigoals/src/navigoal_core.cpp
 *
 * @brief Creates a node for controlling the goals given to xbot
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
#include <stdlib.h>
#include <xbot_navigoals/RobotStatus.h>
#include <xbot_navigoals/SignStatus.h>
#include <xbot_navigoals/AudioStatus.h>
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
															 current_position(0),
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
	navigoal_status_subscriber = nh.subscribe("/move_base/result", 10,&NaviGoalCore::naviGoalStatus, this);
	pad_finished_goal_subscriber = nh.subscribe("/pad_sign_completion",1,&NaviGoalCore::receiveSignStatus, this);
	pad_audio_subscriber = nh.subscribe("/pad_audio_status",1,&NaviGoalCore::receiveAudioStatus,this);
	/*********************
	 ** Publishers
	 **********************/
	plans_publisher_ = nh.advertise<nav_msgs::Path>("/navi_plans",10);
	execute_goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
	marker_goals_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("marker_goals",0);
	reached_subgoal_publisher = nh.advertise<xbot_msgs::NaviState>("reached_subgoal",1);
	arrive_goal_publisher = nh.advertise<xbot_navigoals::RobotStatus>("/robot_status",10); 
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


	start_point.header.stamp = ros::Time();
	start_point.header.seq = 0;
	std::string s = "map";
	start_point.header.frame_id  = s;
  start_point.pose.position.x = 5.0;
  start_point.pose.position.y = 0.0;
	start_point.pose.position.z = 0.0;
	start_point.pose.orientation.x = 0.0;
	start_point.pose.orientation.y = 0.0;
  start_point.pose.orientation.z = 0.0;
  start_point.pose.orientation.w = 1.0;

	//first publish a initial event
	 xbot_navigoals::RobotStatus initStatus;
	 initStatus.id = -1;
	 initStatus.ismoving = false;
	 arrive_goal_publisher.publish(initStatus);


	is_started = false;
	// start keyboard input thread
	thread.start(&NaviGoalCore::keyboardInputLoop, *this);
	return true;
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop;
 *
 * It also process ros functions as well as aborting when requested.
 */
void NaviGoalCore::spin()
{
	ros::Rate loop_rate(10);

	while (!quit_requested&&ros::ok())
	{

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
	 *
	 */
	switch (c)
	{        
		case 'n':
		{
			createGoalSerials();
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

//topic :  /move_base/result
//when movebase reached/unreached a goal,this function was called 
//status:2    canceled
//status:3    success
//status:4    Fail to find a path
void NaviGoalCore::naviGoalStatus(const move_base_msgs::MoveBaseActionResult& result)
{
  //reached goal success
  switch (result.status.status) {
  case 3:{
    xbot_msgs::NaviState state;
    state.subgoal_index = (++current_position < num_goals)?current_position:0;
    current_position = (current_position <num_goals)?current_position:0;
    reached_subgoal_publisher.publish(state);
    ROS_INFO("Current Position id : %d .Watting for Callback from Pad ....",current_position);
    //send message to pad ,let pad to recognize face or play voice
    xbot_navigoals::RobotStatus robotStatus;
    robotStatus.id = current_position;
    robotStatus.ismoving = false;
    arrive_goal_publisher.publish(robotStatus);
    //have finished a path
    if(current_position == num_goals-1){
      if(is_started){
        ROS_INFO("Finished a path.....");
        is_started = false;
      }
    }
    break;
  }
  case 4:
    break;
  default: break;
  }
}

// Interact with xbot_head 
// subscribed topic "pad_sign_completion" callback function..
void NaviGoalCore::receiveSignStatus(const xbot_navigoals::SignStatus& signStatus){
	if(signStatus.complete == true){
	   if(signStatus.success == true){
			ROS_INFO("Face Recognition Success at Position : %d \n",current_position);
			//means  a person's face has been corretly recognized
			//turn to next point 
       execute_goal_publisher_.publish(goals[current_position+1]);
//			 start_point = goals[current_position];
	   }else {
			ROS_INFO("Face Recognition Failure at Position :%d \n",current_position);
			//face recognized failure			 
       execute_goal_publisher_.publish(goals[current_position+1]);
//			 start_point = goals[current_position];
	   } 
	}else{
		//cannot connect to face recognition server ,or timeout
		ROS_INFO("Face Recognition Server Connection Timeout at Position :%d \n",current_position);
     execute_goal_publisher_.publish(goals[current_position+1]);
//		 start_point = goals[current_position];
	}

}

//subscribed topic "pad_audio_status" callback function
void NaviGoalCore::receiveAudioStatus(const xbot_navigoals::AudioStatus& audioStatus){
	if(audioStatus.iscomplete == true){
    ROS_INFO(" NO.%d audio completed.",audioStatus.id);
    if(current_position < num_goals-1){
      execute_goal_publisher_.publish(goals[current_position+1]);
    }
    else{
      execute_goal_publisher_.publish(goals[0]);
    }

	}

}

//2D Nav Goal tool clicked callback function..
void NaviGoalCore::clickedGoalReceived(const geometry_msgs::PoseStamped &pose)
{
	if(!click_goal_finished){
		if(num_goals<MAX_GOAL_NUM){
			goals[num_goals]=pose;
			num_goals++;
			ROS_INFO("addded a new goal:\n position:\n x: %f\n y: %f\n z:%f\n  There are %d/%d goals now!\n",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,num_goals,MAX_GOAL_NUM);
			//ROS_INFO("goal header: seq:%d ,frame_id:%s \n",pose.header.seq,pose.header.frame_id.c_str());
			//ROS_INFO("goal orientation: x :%f ,y:%f ,z:%f , w:%f \n",pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
			marker.header.stamp = ros::Time();
			marker.id = num_goals;

			marker.pose = pose.pose;
			arraymarker.markers.push_back(marker);
			marker_goals_publisher_.publish( arraymarker );

			if(num_goals>1){
				make_plan_srv.request.start = goals[num_goals-2];
				make_plan_srv.request.goal = goals[num_goals-1];
				make_plan_srv.request.tolerance = 0.5;

				//request the path culculation 
				if(serviceClient.call(make_plan_srv)){
					if(!make_plan_srv.response.plan.poses.empty()){
		//               ROS_INFO(make_plan_srv.response.plan.header.frame_id);
						  int size = make_plan_srv.response.plan.poses.size();
						  for(int i=0;i< size ;i++){
								navi_path.poses.push_back(make_plan_srv.response.plan.poses[i]);
						   }
						   ROS_INFO("Got plan in function : clickedGoalReceived()  -- plan size: %d",size);
					}else{
							ROS_WARN("Got empty plan");
					}
				}else{
						ROS_ERROR("Can not call service!");
				}
      }/*else{
				//means this is the first clicked point 
				first_click_point = pose;
	
				make_plan_srv.request.start = start_point;
				make_plan_srv.request.goal = first_click_point;
				make_plan_srv.request.tolerance = 0.5;
				if(serviceClient.call(make_plan_srv)){
					if(!make_plan_srv.response.plan.poses.empty()){
						  int size = make_plan_srv.response.plan.poses.size();
						  for(int i=0;i< size ;i++){
								navi_path.poses.push_back(make_plan_srv.response.plan.poses[i]);
						   }
						   ROS_INFO("Got plan in clickedGoalReceived()  -- plan size: %d",size);
					}else{
							ROS_WARN("Got empty plan");
					}
				}else{
						ROS_ERROR("Can not call service!");
				}
      }*/
			plans_publisher_.publish(navi_path);
			//ROS_INFO("clickedGoalReceived  ----   Published navi_path  size :%d",navi_path.poses.size());
		}else{
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

		arraymarker.markers.clear();
    marker.header.stamp = ros::Time();
    marker.id = num_goals;
    marker.pose = start_point.pose;
    arraymarker.markers.push_back(marker);
    marker_goals_publisher_.publish( arraymarker );
    goals[0] = start_point;
    num_goals=1;
    current_position = 0;
		navi_path.poses.clear();
    ROS_INFO("Initialized, start point has been added into goals, current goal number:1,please enter your goals.");
	}else{
    ROS_INFO("You are clicking a goal serial, want to clear them? please press 'c' key!");
	}


}

/*** clear all goals..
 */
void NaviGoalCore::clearAllGoals()
{
	if(!click_goal_finished)
	{
		current_position = 0;
		num_goals=0;
		arraymarker.markers.clear();
		ROS_INFO_STREAM("All of the goals are cleared");
	}

}

//keyEvent:  f
//when goals were selected completely
//this callback called
void NaviGoalCore::completeGoals()
{
	
	make_plan_srv.request.start = goals[num_goals-1];
	make_plan_srv.request.goal = goals[0];
	make_plan_srv.request.tolerance = 0.5;

	if(serviceClient.call(make_plan_srv))
	{
		if(!make_plan_srv.response.plan.poses.empty())
		{
			 int size = make_plan_srv.response.plan.poses.size();
			for(int i=0; i<size; i++)
			{
				navi_path.poses.push_back(make_plan_srv.response.plan.poses[i]);
			}
			ROS_INFO("Got plan in function : completeGoals(),  plan size: %d",size);
		}
		else
		{
			ROS_WARN("Got empty plan");
		}
	}
	
	plans_publisher_.publish(navi_path);

	//push the first point(0,0) to the goals array,
	//in order to make xbot move to the origin point after finishing a path 
//	goals[num_goals++] = first_click_point;
	
	click_goal_finished = true;
	reached_goal = false;
	
//  execute_goal_publisher_.publish(goals[1]);

	ROS_INFO_STREAM("Click goals finished!");

}


} // namespace navigoal_core
