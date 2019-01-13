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
*@file turtlebot.cpp
*@author Sudarshan Raghunathan
*@brief  Functions definitions for turtlebot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlebot.hpp"
#include "line_follower_turtlebot/pos.h"
#include "line_follower_turtlebot/col.h"

void turtlebot::dir_sub(line_follower_turtlebot::pos msg) {
    turtlebot::dir = msg.direction;
}

void turtlebot::col_sub(line_follower_turtlebot::col msg) {
    turtlebot::col_flag = msg.collision_flag;
}
void turtlebot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {
    // If direction is left
    if (turtlebot::dir == 0) {
        velocity.linear.x = turtlebot::proximity();
        velocity.angular.z = 0.2;
        pub.publish(velocity);
        rate.sleep();
        if(turtlebot::col_flag != 0)
            ROS_INFO_STREAM("Turning Left");
        else
            ROS_INFO_STREAM("Risk of Collision: Stop");
    }
    // If direction is straight
    if (turtlebot::dir == 1) {
        velocity.linear.x = turtlebot::proximity();;
        velocity.angular.z = 0;
        pub.publish(velocity);
        rate.sleep();
        if(turtlebot::col_flag != 0)
            ROS_INFO("Straight at speed: %f", velocity.linear.x);
        else
            ROS_INFO_STREAM("Risk of Collision: Stop");
    }
    // If direction is right
    if (turtlebot::dir == 2) {
        velocity.linear.x = turtlebot::proximity();
        velocity.angular.z = -0.2;
        pub.publish(velocity);
        rate.sleep();
        if(turtlebot::col_flag != 0)
            ROS_INFO_STREAM("Turning Right");
        else
            ROS_INFO_STREAM("Risk of Collision: Stop");
    }
    // If robot has to search
    if (turtlebot::dir == 3) {
        velocity.linear.x = 0;
        velocity.angular.z = 0.25;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
}

float turtlebot::proximity(void)
{
    if(turtlebot::col_flag == 0)
        return STOP;
    if(turtlebot::col_flag == 1)
        return SLOW;
    if(turtlebot::col_flag == 2)
        return MEDIUM;
    if(turtlebot::col_flag == 3)
        return FAST;

    return FAST;
}