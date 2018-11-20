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
 *  @file   walker.hpp
 *
 *  @author   Kapil Rawal
 *
 *  @copyright   BSD License
 *
 *  @brief   ROS turtle bot walker 
 *
 *  @section   DESCRIPTION
 *
 *  Walker header file containing class walker and it's declerations
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 *
 *  @brief   Created class Walker.
 *
 */
class Walker {
 private:
  /// Declared a variable for collision
  bool collision;

  /// Declared a variable for the velocities
  geometry_msgs::Twist msg;

  /// Created a node handle
  ros::NodeHandle n;

  /// Variable for velocity publish
  ros::Publisher velocityPublish;

  /// Subscribe to the laser scan topic
  ros::Subscriber lasersub;

 public:
  /// Constructor for Walker
  Walker();
  
  /// Destructor of walker
  ~Walker();

  /// Running the robot
  void runTurtleBot();
  
  /// Callback function for Walker
  void laserCall(const sensor_msgs::LaserScan::ConstPtr& msg);

};

#endif  // INCLUDE_WALKER_HPP_

