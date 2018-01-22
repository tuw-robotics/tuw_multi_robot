/* Copyright (c) 2017, TU Wien
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY TU Wien ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL TU Wien BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TUW_MULTI_ROBOT_ROUTE_TO_PATH_H
#define TUW_MULTI_ROBOT_ROUTE_TO_PATH_H


// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/SegmentPath.h>
#include <tuw_multi_robot_route_to_path/RobotRouteToPath.h>
#include <tuw_multi_robot_route_to_path/RobotStateObserver.h>

#include <memory>

namespace  tuw_multi_robot_route_to_path
{
    class MultiRobotRouteToPathNode
    {
            //special class-member functions.
        public   : MultiRobotRouteToPathNode(ros::NodeHandle& n);

            //ROS:
        public   : ros::NodeHandle                          n_;      ///< Node handler to the root node
        public   : ros::NodeHandle                          n_param_;///< Node handler to the current node
        public   : std::unique_ptr<ros::Rate>               rate_;

        private  : void publishPath(std::vector<Eigen::Vector3d> _p, int _topic);

        private  : std::vector<ros::Publisher>              pubPath_;
        private  : std::vector<ros::Subscriber>             subSegPath_;
        private  : std::vector<ros::Subscriber>             subOdometry_;


            // ROS Topic names
        private  : std::string                              topic_path_;
        private  : std::string                              topic_seg_path_;
        private  : std::string                              topic_odom_;
        private  : std::vector<std::string>                 robot_names_;


        private  : void subOdomCb(const ros::MessageEvent<nav_msgs::Odometry const>& _event, int _topic);
        private  : void subSegPathCb(const ros::MessageEvent<tuw_multi_robot_msgs::SegmentPath const>& _event, int _topic);

        private : std::vector<RobotRouteToPath>             converter_;
        private : std::vector<RobotStateObserver>           observer_;
        private : int                                       no_robots_;
        private : std::vector<int>                          robot_steps_;
    };

}

#endif

