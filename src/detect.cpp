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
 *@file   detect.cpp
 *@author Sudarshan Raghunathan
 *@brief  Ros Nod to subscribe to turtlebot images and perform image processing to detect line
 */
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "linedetect.hpp"
#include "collisiondetect.hpp"
#include "line_follower_turtlebot/pos.h"
#include "line_follower_turtlebot/col.h"

/**
 *@brief Main function that reads image from the turtlebot and provides direction to move using image processing
 *@param argc is the number of arguments of the main function
 *@param argv is the array of arugments
 *@return 0
*/

int main(int argc, char **argv) {
    // Initializing node and objects
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    LineDetect det;
    CollisionDetect coldet;

    // Creating Subscribers
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw",
        1, &LineDetect::imageCallback, &det);

    ros::Subscriber sub2 = n.subscribe("/scan",
        1, &CollisionDetect::laserScanMsgCallBack, &coldet);
    //Creating Publishers
    ros::Publisher dirPub = n.advertise<
    line_follower_turtlebot::pos>("direction", 1);
        line_follower_turtlebot::pos msg_dir;

    ros::Publisher colPub = n.advertise<
    line_follower_turtlebot::col>("collision_flag", 1);
        line_follower_turtlebot::col msg_col;

    while (ros::ok()) {
        if (!det.img.empty()) {
            // Perform image processing
            det.img_filt = det.Gauss(det.img);
            msg_dir.direction = det.colorthresh(det.img_filt);
            // Imminent collision flag determination
            msg_col.collision_flag = coldet.distance_flag;

            // Publish direction and colision flag message
            dirPub.publish(msg_dir);
            colPub.publish(msg_col);
            }
        ros::spinOnce();
    }
    // Closing image viewer
    cv::destroyWindow("Turtlebot View");
}
