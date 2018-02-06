/**
 * @file /xbot_navigoals/src/main.cpp
 *
 * @brief Executable code for the navigoals node.
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/navigoal_core/navigoal_core.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using navigoal_core::NaviGoalCore;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xbot_navigoals");
  NaviGoalCore navigoal;
  if (navigoal.init())
  {
    navigoal.spin();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise NaviGoalCore!");
  }

  ROS_INFO_STREAM("Program exiting");
  return 0;
}
