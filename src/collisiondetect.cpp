/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file collisiondetect.cpp
*@author Pablo Ferreiro
*@brief  Class collisiondetect's function definitions
*/
#include "collisiondetect.hpp"
#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "line_follower_turtlebot/col.h"


void CollisionDetect::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  
  scan_data_[CENTER] = msg->ranges.at(msg->ranges.size()/2);
  scan_data_[LEFT] = msg->ranges.at(0);
  scan_data_[RIGHT] = msg->ranges.at(msg->ranges.size()-1);
  
  for(int angle = 0; angle < 3 ; angle++)
  {
    if (std::isnan(msg->ranges.at(angle)))
    {
      scan_data_[angle] = msg->range_max; 
    }
  }
  
  
  ROS_DEBUG_THROTTLE(2, "Left laser distance: %f", scan_data_[LEFT]);
  ROS_DEBUG_THROTTLE(2, "Center laser distance: %f", scan_data_[CENTER]);
  ROS_DEBUG_THROTTLE(2, "Right laser distance: %f", scan_data_[RIGHT]);

  if (scan_data_[CENTER] < check_forward_dist_ || scan_data_[LEFT] < check_side_dist_ || scan_data_[RIGHT] < check_side_dist_ )
    collision_flag = 1;
  else
    collision_flag = 0;
}


