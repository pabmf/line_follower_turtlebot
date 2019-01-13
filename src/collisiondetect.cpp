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
  //Find obstacle distance at 0, -30 and +30 degres
  scan_data_[CENTER] = msg->ranges.at(msg->ranges.size()/2);
  scan_data_[RIGHT] = msg->ranges.at(0);
  scan_data_[LEFT] = msg->ranges.at(msg->ranges.size()-1);
  
  //Replace NaN values for maximum distance range
  for(int angle = 0; angle < 3 ; angle++)
  {
    if (std::isnan(scan_data_[angle]))
    {
      scan_data_[angle] = msg->range_max; 
    }
  }
  
  //Printing distance for debugging purposes
  ROS_DEBUG_THROTTLE(2, "Left laser distance: %f", scan_data_[LEFT]);
  ROS_DEBUG_THROTTLE(2, "Center laser distance: %f", scan_data_[CENTER]);
  ROS_DEBUG_THROTTLE(2, "Right laser distance: %f", scan_data_[RIGHT]);


  //Finding closest obstacle distance
  float smallest = scan_data_[0];
  for(int angle = 1 ; angle < 3 ; angle++)
  {
    if (scan_data_[angle] < smallest)
      smallest = scan_data_[angle];
  }

  ROS_DEBUG_THROTTLE(2, "Smallest distance: %f", smallest);

  //Setting obstacle distance flag to be published
  if(smallest <= STOP)
      {distance_flag = 0;
          ROS_DEBUG_THROTTLE(2, "Obstacle flag: STOP");}
  else
    if(smallest <= NEAR)
      {distance_flag = 1;
          ROS_DEBUG_THROTTLE(2, "Obstacle flag: NEAR");}
  else 
    if(smallest <= MIDDLE)
      {distance_flag = 2;
          ROS_DEBUG_THROTTLE(2, "Obstacle flag: MIDDLE");}
  else
    if(smallest <= FAR)
      {distance_flag = 3;
          ROS_DEBUG_THROTTLE(2, "Obstacle flag: FAR");}
  else
      {distance_flag = 4;
          ROS_DEBUG_THROTTLE(2, "Obstacle flag: VERY FAR");}

  

}


