/**
 * BSD 3-Clause License


 * Copyright (c) 2018, KapilRawal
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *  @copyright (c) BSD
 *
 *  @file   walker.cpp
 *
 *  @author   Kapil Rawal
 *
 *  @copyright   BSD License
 *
 *  @brief   ROS walker node
 *
 *  @section   DESCRIPTION
 *
 *  walker.cpp is the defination of class Walker
 *
 */

#include <iostream>
#include "walker.hpp"

/**
 *
 *  @brief   Constructs the object
 *
 */
Walker::Walker() {
  /// initialise the flag
  collision = false;
  /// Publish the velocity to cmd_vel_mux/input/navi
  velocityPublish = n.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/navi", 1000);
  /// Subcribe to the /scan topic and use the laserCallback method
  lasersub = n.subscribe < sensor_msgs::LaserScan
      > ("/scan", 300, &Walker::laserCall, this);
  /// stop the turtlebot
  velocityPublish.publish(msg);
  ROS_INFO("Configuration setting done");
}

/**
 * 
 * @brief   Destroy the objects
 *
 */
Walker::~Walker() {
  /// stopping the bot
  velocityPublish.publish(msg);
}

/**
 * 
 * @brief   Callback from laser scan
 *
 * @param   msg The message published 
 *
 */
void Walker::laserCall(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.5) {
      collision = true;
      return;
    }
  }
  collision = false;
}


/**
 *
 * @brief   Running the Robot
 *
 */
void Walker::runTurtleBot() {
  /// Rate of loop running
  ros::Rate loop_rate(10);
  /// Keep running till ros is running ok
  while (ros::ok()) { 
   /// Finding the obstacle
    if (collision) {
      /// Obstacle detected
      ROS_INFO("obstacle detected , Turning away");

      /// Stop the robot
      msg.linear.x = 0.0;

      /// Rotating the robot
      msg.angular.z = 0.5;
    } else {
      
      /// Stop Rotating
      msg.angular.z = 0.0;

      /// Set forward speed of the Turtle robot
      msg.linear.x = 0.5;

      ROS_INFO("NO obstacle, Moving ahead");

     }

    /// Publish the twist message as broadcast
    velocityPublish.publish(msg);

    /// for subscriber node
    ros::spinOnce();

    /// when not publishing in sleep mode to match the publish rate
    loop_rate.sleep();
  }
}

