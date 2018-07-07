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
#include <ros/ros.h>
#include <tuw_multi_robot_route_to_path/LocalBehaviorControllerNode.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_behavior_controller_node");  /// initializes the ros node with default name
  ros::NodeHandle n;

  tuw_multi_robot_route_to_path::LocalBehaviorControllerNode ctrl(n);
  ros::Rate r(2);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    ctrl.publishRobotInfo();
  }

  return 0;
}

namespace tuw_multi_robot_route_to_path
{
LocalBehaviorControllerNode::LocalBehaviorControllerNode(ros::NodeHandle &n)
  : n_(n), n_param_("~"), robot_name_(std::string("robot_0"))
{
  observer_ = RobotStateObserver();
  robot_step_ = -1;
  robot_route_ = tuw_multi_robot_msgs::Route();
  
  n_param_.param("robot_name", robot_name_, robot_name_);
  ROS_INFO("robot name = %s", robot_name_.c_str());

  n_param_.param("robot_radius", robot_radius_, robot_radius_);
  robotDefaultRadius_ = 0.3;
  n_param_.param("robot_default_radius", robotDefaultRadius_, robotDefaultRadius_);

  topic_path_ = "path";
  n_param_.param("path_topic", topic_path_, topic_path_);

  topic_route_ = "route";
  n_param_.param("route_topic", topic_route_, topic_route_);

  topic_robot_info_ = "/robot_info";
  n_param_.param("robotInfo_topic", topic_robot_info_, topic_robot_info_);
  
  topic_pose_ = "/pose";
  n_param_.param("pose_topic", topic_pose_, topic_pose_);

  subPose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot_name_ + "/" + topic_pose_, 1,
                                                                   &LocalBehaviorControllerNode::subPoseCb, this);
  subRobotInfo_ = n.subscribe<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, 10000, &LocalBehaviorControllerNode::subRobotInfoCb, this);

  subRoute_ = n.subscribe<tuw_multi_robot_msgs::Route>(robot_name_ + "/" + topic_route_, 1,
                                                       &LocalBehaviorControllerNode::subRouteCb, this);
  
  pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, 10000);
  pubPath_ = n.advertise<nav_msgs::Path>(robot_name_ + "/" + topic_path_, 100);
}

void LocalBehaviorControllerNode::publishPath(std::vector<Eigen::Vector3d> _p)
{
  nav_msgs::Path path;
  ros::Time now = ros::Time::now();
  path.header.stamp = now;
  path.header.frame_id = "map";
  
  for(auto&& p : _p)
  {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now;
    ps.header.frame_id = "map";
     
    ps.pose.position.x = p[0];
    ps.pose.position.y = p[1];
    
    Eigen::Quaternion<float> q;
    q = Eigen::AngleAxisf(p[2], Eigen::Vector3f::UnitZ());
    
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    
    path.poses.push_back(ps);
  }
//  ROS_INFO("robot: %s, publishPath path size = %d", robot_name_.c_str(), path.poses.size());
  pubPath_.publish(path);
}

void LocalBehaviorControllerNode::updatePath()
{
  if(robot_route_.segments.size() == 0)
    return;
    
  std::vector<Eigen::Vector3d> path;
  std::vector<PathSegment> seg_path;
  
  auto&& seg = robot_route_.segments.begin();
  seg += robot_step_;
  
  // go through all segments in the route
  for(; seg != robot_route_.segments.end(); seg++)
  {
    PathSegment path_seg;
    path_seg.start[0] = seg->start.position.x;
    path_seg.start[1] = seg->start.position.y;
    path_seg.goal[0] = seg->end.position.x;
    path_seg.goal[1] = seg->end.position.y;
    path_seg.width = seg->width;
    
    seg_path.emplace_back(path_seg);
    
    bool valid = true;
    
    // go through all preconditions and check if they are fulfilled
    for(auto&& prec : seg->preconditions)
    {
      if(prec.robot_id != robot_name_ && robot_steps_[prec.robot_id] < prec.current_route_segment)
      {
        valid = false;
      }
    }
    
    // add segments to path as long as the prec. are fulfilled
    if(valid)
    {
      Eigen::Vector3d pose;
      
      double r, p, y;
      tf::Quaternion q(seg->end.orientation.x, seg->end.orientation.y, seg->end.orientation.z, seg->end.orientation.w);
      tf::Matrix3x3(q).getRPY(r, p, y);
      
      pose[0] = seg->end.position.x;
      pose[1] = seg->end.position.y;
      pose[2] = y;
      
      path.emplace_back(pose);
    }
  }
  
  //observer_.init(seg_path);
  
  if(path.size() > 1)
  {
    publishPath(path);
  }
}

void LocalBehaviorControllerNode::subRouteCb(const tuw_multi_robot_msgs::Route::ConstPtr &_route)
{
  robot_route_ = *_route;
  
  if(robot_route_.segments.size() == 0)
    return;
  
  // clear when new route arrives
  robot_step_ = 0;
  
  std::vector<PathSegment> seg_path;
  
  // go through all segments in the route
  for(auto&& seg : robot_route_.segments)
  {
    PathSegment path_seg;
    path_seg.start[0] = seg.start.position.x;
    path_seg.start[1] = seg.start.position.y;
    path_seg.goal[0] = seg.end.position.x;
    path_seg.goal[1] = seg.end.position.y;
    path_seg.width = seg.width;
    
    seg_path.emplace_back(path_seg);
  }
  
  observer_.init(seg_path);
}

void LocalBehaviorControllerNode::subPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose)
{
  robot_pose_ = _pose->pose;
  bool changed = true;
  
  robot_step_ = observer_.getStep(Eigen::Vector2d(_pose->pose.pose.position.x, _pose->pose.pose.position.y), changed);
}

void LocalBehaviorControllerNode::subRobotInfoCb(
    const tuw_multi_robot_msgs::RobotInfo_<std::allocator<void> >::ConstPtr &_robot_info)
{
  robot_steps_[_robot_info->sync.robot_id] = _robot_info->sync.current_route_segment;
  updatePath();
}

void LocalBehaviorControllerNode::publishRobotInfo()
{
  tuw_multi_robot_msgs::RobotInfo ri;
  ri.header.stamp = ros::Time::now();
  ri.robot_name = robot_name_;
  ri.pose = robot_pose_;
  ri.shape = ri.SHAPE_CIRCLE;
  ri.shape_variables.push_back(robot_radius_);
  ri.sync.robot_id = robot_name_;
  ri.sync.current_route_segment = robot_step_;
  ri.mode = ri.MODE_NA;
  ri.status = ri.STATUS_STOPPED;  // TODO
  ri.good_id = ri.GOOD_NA;

  pubRobotInfo_.publish(ri);
}
}  // namespace tuw_multi_robot_route_to_path
