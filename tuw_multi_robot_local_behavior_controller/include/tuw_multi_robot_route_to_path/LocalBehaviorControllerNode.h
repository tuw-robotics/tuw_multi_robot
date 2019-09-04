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

#ifndef LOCAL_BEHAVIOR_CONTROLLER_NODE_H
#define LOCAL_BEHAVIOR_CONTROLLER_NODE_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tuw_nav_msgs/ControllerState.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <tuw_multi_robot_route_to_path/RobotRouteToPath.h>
#include <tuw_multi_robot_route_to_path/RobotStateObserver.h>
#include <tuw_multi_robot_route_to_path/RouteProgressMonitor.h>
#include <actionlib/client/simple_action_client.h>

#include <tuw_local_controller_msgs/ExecutePathAction.h>

#include <memory>

namespace tuw_multi_robot_route_to_path
{
class LocalBehaviorControllerNode
{
public:
  LocalBehaviorControllerNode(ros::NodeHandle &n);

  // ROS:
  ros::NodeHandle n_;        ///< Node handler to the root node
  ros::NodeHandle n_param_;  ///< Node handler to the current node
  std::unique_ptr<ros::Rate> rate_;

  void publishRobotInfo();
private:
  void updatePath();

  ros::Publisher pubPath_;
  ros::Publisher pubRobotInfo_;
  ros::Subscriber subRoute_;
  ros::Subscriber subCtrlState_;
  ros::Subscriber subPose_;
  ros::Subscriber subRobotInfo_;

  // ROS Topic names
  double update_rate_;
  std::string frame_id_;
  std::string robot_name_;
  double robot_radius_;
  double robotDefaultRadius_ = 0.6;
  
  geometry_msgs::PoseWithCovariance robot_pose_;
  tuw_multi_robot_msgs::Route route_;
  tuw_multi_robot_msgs::RobotInfo robot_info_;
  tuw_nav_msgs::ControllerState ctrl_state_;
  nav_msgs::Path path_;
  
  int path_segment_start; /// route segment idx used to define the path start point
  int path_segment_end;   /// route segment idx used to define the path end point

  void subCtrlCb(const tuw_nav_msgs::ControllerStateConstPtr& msg);
  void subPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& _pose);
  void subRobotInfoCb(const tuw_multi_robot_msgs::RobotInfo::ConstPtr& _robot_info);
  void subRouteCb(const tuw_multi_robot_msgs::Route::ConstPtr& _route);

  //RobotRouteToPath converter_;
  //RobotStateObserver observer_;
  int robot_step_;
  std::map<std::string, int> robot_steps_;
  
  tuw::RouteProgressMonitor progress_monitor_;

  actionlib::SimpleActionClient<tuw_local_controller_msgs::ExecutePathAction> client;
};

}  // namespace tuw_multi_robot_route_to_path

#endif  // LOCAL_BEHAVIOR_CONTROLLER_NODE_H
