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
*@file collisiondetect.hpp
*@author Pablo Ferreiro
*@brief Header file for class collisiondetect
*/

#pragma once
#include <vector>
#include "ros/ros.h"
#include "line_follower_turtlebot/col.h"
#include <sensor_msgs/LaserScan.h>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define RIGHT 0
#define CENTER 1
#define LEFT  2

#define FAR 5
#define MIDDLE 3
#define NEAR 1.5
#define STOP 0.5

/**
*@brief Collision Detect class contains all the functions for laser scan processing and or image depth processing and collision avoidance
*/
class CollisionDetect{
 public:
   int distance_flag;  /// distance Flag to be published 

/**
*@brief Callback used to subscribe to the scan topic from the Turtlebot
*@param msg is the laser scan message for ROS
*@return none
*/
    void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);


 private:
    double scan_data_[3] = {0.0,0.0,0.0};

};
