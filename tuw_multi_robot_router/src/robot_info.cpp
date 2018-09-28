/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#define POT_HIGH 1.0e10

#include <tuw_global_router/robot_info.h>
#include <tuw_global_router/mrr_utils.h>
#include <tuw_global_router/srr_utils.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <boost/regex.hpp>
#include <tf/tf.h>

//TODO add Weights from robots...


namespace multi_robot_router {


void RobotInfo::callback_odom ( const nav_msgs::Odometry &msg ) {
    // ToDo
}

bool RobotInfo::compareName(const std::shared_ptr<RobotInfo> robot) const{
        std::string a = boost::regex_replace( this->robot_name,  boost::regex("[^0-9]*([0-9]+).*"), std::string("\\1"));
        std::string b = boost::regex_replace( robot->robot_name, boost::regex("[^0-9]*([0-9]+).*"), std::string("\\1"));
        /// @ToDo check if it crashes if the robot name holds no number 
        return std::stoi ( a ) <  std::stoi ( b );    
}
void RobotInfo::initTopics ( ros::NodeHandle &n,  bool robot_name_as_namespace) {

    //Not existant subscribe robots
    std::string ns;
    if( robot_name_as_namespace ) {
        ns = robot_name + "/";
    }
    std::string topic_odom_name = ns + "odom";
    ROS_DEBUG ( "Multi Robot Router: subscribing to %s", topic_odom_name.c_str() );
    subOdom_ = n.subscribe ( topic_odom_name, 1, &RobotInfo::callback_odom, this );

    std::string topic_path_name = ns + "path_unsynced";
    ROS_DEBUG ( "Multi Robot Router: advertising on %s", topic_path_name.c_str() );
    pubPaths_ = n.advertise<nav_msgs::Path> ( topic_path_name, 1, true );


    std::string topic_route_name = ns + "route";
    ROS_DEBUG ( "Multi Robot Router: advertising on %s", topic_route_name.c_str() );
    pubRoute_ = n.advertise<tuw_multi_robot_msgs::Route> ( topic_route_name, 1, true );
}

void RobotInfo::updateInfo ( const tuw_multi_robot_msgs::RobotInfo &info ) {
    tuw_multi_robot_msgs::RobotInfo &des = *this;
    des = info;
}

size_t RobotInfo::findIdx ( const std::vector<std::shared_ptr<RobotInfo> > &data, const std::string &name ) {
    for ( size_t i = 0; i < data.size(); i++ ) {
        if ( data[i]->robot_name == name ) return i;
    }
    return data.size();
}

std::vector<std::shared_ptr<RobotInfo> >::iterator RobotInfo::findObj ( std::vector<std::shared_ptr<RobotInfo> > &data, const std::string &name ) {
    return std::find_if ( data.begin(), data.end(),  [&] ( auto & r ) {
        return r->robot_name == name;
    } );
}

Eigen::Vector3d RobotInfo::getPose() const {
    double roll, pitch, yaw;
    tf::Quaternion q ( pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w );
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    Eigen::Vector3d p ( pose.pose.position.x, pose.pose.position.y, yaw );
    return p;
}

float RobotInfo::radius() const {
    if ( shape == SHAPE_CIRCLE ) {
        return shape_variables[0];
    }

    return -1;
}

RobotInfo::Online RobotInfo::getOnlineStatus() const {
    return online_;
}

void RobotInfo::updateOnlineStatus ( const float _updateTime ) {
    if ( activeTime_ > 0 )
        activeTime_ -= _updateTime;

    if ( activeTime_ < 0 )
        activeTime_ = 0;

    if ( activeTime_ == 0 && online_ != Online::fixed )
        online_ = Online::inactive;
}

} // namespace multi_robot_router
