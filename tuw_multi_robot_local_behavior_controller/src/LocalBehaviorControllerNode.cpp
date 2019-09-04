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

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "local_behavior_controller_node" ); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_multi_robot_route_to_path::LocalBehaviorControllerNode ctrl ( n );

    return 0;
}

namespace tuw_multi_robot_route_to_path {
LocalBehaviorControllerNode::LocalBehaviorControllerNode ( ros::NodeHandle &n )
        : n_(n), n_param_("~"), client(n, "execute_path")
{
    robot_step_ = -1;
    route_ = tuw_multi_robot_msgs::Route();
    path_segment_end = 0;
    path_segment_start = 0;

    n_param_.param<std::string> ( "robot_name", robot_name_, "r0" );
    ROS_INFO ( "robot name = %s", robot_name_.c_str() );

    n_param_.param<double> ( "robot_radius", robot_radius_, robot_radius_ );

    n_param_.param<double> ( "robot_default_radius", robotDefaultRadius_, 0.3 );

    n_param_.param<std::string> ( "frame_id", frame_id_, "map" );

    n_param_.param<double> ( "update_rate", update_rate_, 1.0 );

    subCtrlState_ = n.subscribe<tuw_nav_msgs::ControllerState> ( "state_trajectory_ctrl", 1,  &LocalBehaviorControllerNode::subCtrlCb, this );
    subPose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped> ( "pose", 1,  &LocalBehaviorControllerNode::subPoseCb, this );
    subRobotInfo_ = n.subscribe<tuw_multi_robot_msgs::RobotInfo> ( "robot_info", 10000, &LocalBehaviorControllerNode::subRobotInfoCb, this );

    subRoute_ = n.subscribe<tuw_multi_robot_msgs::Route> ( "route", 1, &LocalBehaviorControllerNode::subRouteCb, this );

    pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo> ( "robot_info", 10000 );
    pubPath_ = n.advertise<nav_msgs::Path> ( "path", 1 );


    ros::Rate r ( update_rate_ );

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        publishRobotInfo();
    }
}

void LocalBehaviorControllerNode::updatePath() {

    bool valid = true;
    size_t last_active_segment = 0;
    // go through all segments in the route
    for ( size_t i = path_segment_start; i < route_.segments.size(); i++ ) {
        const tuw_multi_robot_msgs::RouteSegment &seg = route_.segments[i];
        // go through all preconditions and check if they are fulfilled
        for ( auto&& prec : seg.preconditions ) {
            std::string other_robot_name = prec.robot_id;
            auto other_robot = robot_steps_.find(other_robot_name);
            if(other_robot == robot_steps_.end()) {
                // no robot info received for this robot
                valid = false;
            } else {
                int other_robot_process_requiered = prec.current_route_segment;
                int other_robot_process_received = robot_steps_[other_robot_name];
                if(other_robot_process_received < other_robot_process_requiered){
                    // robot is not far enoth to process further
                    valid = false;
                }
            }
        }

        // add segments to path as long as the prec. are fulfilled
        if ( valid ) {
            last_active_segment = i;
        } else {
            break;
        }
    }
    if ( last_active_segment > path_segment_end ) {
        path_segment_end = last_active_segment;
        path_segment_start = progress_monitor_.getProgress() + 1;
        geometry_msgs::PoseStamped pose_stamped;
        path_.header = route_.header;
        path_.header.stamp = ros::Time::now(); 
        pose_stamped.header = route_.header;
        pose_stamped.header.stamp = ros::Time::now(); 
            
        path_.poses.clear();
        for ( size_t i = path_segment_start; i <= path_segment_end; i++ ) {
            pose_stamped.pose = route_.segments[i].end;
            path_.poses.push_back(pose_stamped);            
        }

        tuw_local_controller_msgs::ExecutePathGoal goal;
        goal.path = path_;
        pubPath_.publish(path_);
        client.sendGoal(goal);
    }
    
}

void LocalBehaviorControllerNode::subCtrlCb ( const tuw_nav_msgs::ControllerStateConstPtr& msg ) {
    ctrl_state_ = *msg;
}

void LocalBehaviorControllerNode::subRouteCb ( const tuw_multi_robot_msgs::Route::ConstPtr &_route ) {
    route_ = *_route;
    path_segment_end = 0;
    path_segment_start = 0;
    progress_monitor_.init(route_);
}

void LocalBehaviorControllerNode::subPoseCb ( const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose ) {
    robot_pose_ = _pose->pose;
    progress_monitor_.updateProgress(tuw::Point2D(robot_pose_.pose.position.x, robot_pose_.pose.position.y) );
}

void LocalBehaviorControllerNode::subRobotInfoCb ( const tuw_multi_robot_msgs::RobotInfo_<std::allocator<void> >::ConstPtr &_robot_info ) {
    std::string other_robot_name = _robot_info->sync.robot_id;
    int other_robot_process = _robot_info->sync.current_route_segment;
    robot_steps_[other_robot_name] = other_robot_process;
}

void LocalBehaviorControllerNode::publishRobotInfo() {
    updatePath();
    robot_info_.header.stamp = ros::Time::now();
    robot_info_.header.frame_id = frame_id_;
    robot_info_.robot_name = robot_name_;
    robot_info_.pose = robot_pose_;
    robot_info_.shape = robot_info_.SHAPE_CIRCLE;
    robot_info_.shape_variables.resize ( 1 );
    robot_info_.shape_variables[0] =  robot_radius_;
    robot_info_.sync.robot_id = robot_name_;
    robot_info_.sync.current_route_segment = progress_monitor_.getProgress();
    robot_info_.mode = robot_info_.MODE_NA;
    robot_info_.status = robot_info_.STATUS_STOPPED;  // TODO
    robot_info_.good_id = robot_info_.GOOD_NA;

    pubRobotInfo_.publish ( robot_info_ );
}
}  // namespace tuw_multi_robot_route_to_path
