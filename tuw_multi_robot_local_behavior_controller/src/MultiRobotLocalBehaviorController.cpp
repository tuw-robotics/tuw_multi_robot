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
#include <tuw_multi_robot_route_to_path/MultiRobotLocalBehaviorController.h>
#include <tf/transform_datatypes.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "route_to_path"); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_multi_robot_route_to_path::MultiRobotLocalBehaviorController ctrl(n);
    ros::Rate r(20);

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
MultiRobotLocalBehaviorController::MultiRobotLocalBehaviorController(ros::NodeHandle &n) : n_(n),
                                                                                           n_param_("~"),
                                                                                           robot_names_(std::vector<std::string>({"robot_0", "robot_1"}))
{
    n_param_.param("robot_names", robot_names_, robot_names_);
    std::string robot_names_string = "";
    n_param_.param("robot_names_str", robot_names_string, robot_names_string);

    if (robot_names_string.size() > 0)
    {
        robot_names_string.erase(std::remove(robot_names_string.begin(), robot_names_string.end(), ' '), robot_names_string.end());
        std::istringstream stringStr(robot_names_string);
        std::string result;

        robot_names_.clear();

        while (std::getline(stringStr, result, ','))
        {
            robot_names_.push_back(result);
        }
    }
    
    for(int i = 0; i < robot_names_.size(); i++)
    {
      std::cout << "robot_name[" << i << "] = " << robot_names_[i] << std::endl;
    }

    n_param_.param("robot_radius", robot_radius_, robot_radius_);
    robotDefaultRadius_ = 0.3;
    n_param_.param("robot_default_radius", robotDefaultRadius_, robotDefaultRadius_);

    no_robots_ = robot_names_.size();

    ROS_INFO("Subscribing %i robots", no_robots_);

    robot_steps_.resize(no_robots_);
    pubPath_.resize(no_robots_);
    subSegPath_.resize(no_robots_);
    subOdometry_.resize(no_robots_);
    robot_radius_.resize(no_robots_, robotDefaultRadius_);
    robot_pose_.resize(no_robots_);

    n_param_.param<std::string>("path_topic", topic_path_, "path_synced");

    n_param_.param<std::string>("route_topic", topic_route_, "route");

    n_param_.param<std::string>("odom_topic", topic_odom_, "odom");

    n_param_.param<std::string>("robotInfo_topic", topic_robot_info_, "/robot_info");

    n_param_.param<std::string>("frame_map", frame_map_, "map");
    
    for (int i = 0; i < no_robots_; i++)
    {
        converter_.emplace_back(no_robots_, i);
        observer_.emplace_back();
    }

    for (int i = 0; i < no_robots_; i++)
    {
        pubPath_[i] = n.advertise<nav_msgs::Path>(robot_names_[i] + "/" + topic_path_, 100);

        subOdometry_[i] = n.subscribe<nav_msgs::Odometry>(robot_names_[i] + "/" + topic_odom_, 1, boost::bind(&MultiRobotLocalBehaviorController::subOdomCb, this, _1, i));
        subSegPath_[i] = n.subscribe<tuw_multi_robot_msgs::Route>(robot_names_[i] + "/" + topic_route_, 1, boost::bind(&MultiRobotLocalBehaviorController::subSegPathCb, this, _1, i));
    }

    pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, 10);
}

void MultiRobotLocalBehaviorController::subOdomCb(const ros::MessageEvent<const nav_msgs::Odometry> &_event, int _topic)
{
    const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom = _event.getMessage();
    robot_pose_[_topic] = odom->pose;
    Eigen::Vector2d pt(odom->pose.pose.position.x, odom->pose.pose.position.y);

    bool changed = false;
    robot_steps_[_topic] = observer_[_topic].getStep(pt, changed);

    if (changed)
    {
        for (int i = 0; i < no_robots_; i++)
        {
            std::vector<Eigen::Vector3d> newPath = converter_[i].updateSync(robot_steps_, changed);

            if (changed)
                ROS_INFO("new path found %i %lu", i, newPath.size());

            if (changed)
                publishPath(newPath, i);
        }
    }
}

void MultiRobotLocalBehaviorController::publishPath(std::vector<Eigen::Vector3d> _p, int _topic)
{
    nav_msgs::Path path;
    path.header.seq = 0;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_map_;

    for (const Eigen::Vector3d &p : _p)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.seq = 0;
        ps.header.stamp = ros::Time::now();
        ps.header.frame_id = frame_map_;

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

    ROS_INFO("published path %i", _topic);
    pubPath_[_topic].publish(path);
}

int MultiRobotLocalBehaviorController::findRobotId(std::string _name)
{
    for (uint32_t i = 0; i < robot_names_.size(); i++)
    {
        if (robot_names_[i].compare(_name) == 0)
            return i;
    }

    return -1;
}

void MultiRobotLocalBehaviorController::subSegPathCb(const ros::MessageEvent<const tuw_multi_robot_msgs::Route> &_event, int _topic)
{
    const tuw_multi_robot_msgs::Route_<std::allocator<void>>::ConstPtr &path = _event.getMessage();

    std::vector<SyncedPathPoint> localPath;
    std::vector<PathSegment> segPath;

    if (path->segments.size() == 0)
        return;

    for (const tuw_multi_robot_msgs::RouteSegment &seg : path->segments)
    {
        SyncedPathPoint spp;
        PathSegment ps;

        ps.start[0] = seg.start.position.x;
        ps.start[1] = seg.start.position.y;

        ps.goal[0] = seg.end.position.x;
        ps.goal[1] = seg.end.position.y;

        ps.width = seg.width; //Its the radius :D

        double r, p, y;
        tf::Quaternion q(seg.end.orientation.x, seg.end.orientation.y, seg.end.orientation.z, seg.end.orientation.w);
        tf::Matrix3x3(q).getRPY(r, p, y);

        //float angle = atan2(seg.end.position.y - seg.start.position.y, seg.end.position.x - seg.start.position.x);

        spp.p[0] = seg.end.position.x;
        spp.p[1] = seg.end.position.y;
        spp.p[2] = y; //angle;

        for (const tuw_multi_robot_msgs::RoutePrecondition &pc : seg.preconditions)
        {
            PathPrecondition prec;
            prec.robot_no = findRobotId(pc.robot_id);
            prec.step = pc.current_route_segment;

            spp.sync.push_back(prec);
        }

        segPath.push_back(ps);
        localPath.push_back(spp);
    }

    //Todo reset controllers with new Path
    converter_[_topic].init(localPath);
    observer_[_topic].init(segPath);
    std::fill(robot_steps_.begin(), robot_steps_.end(), 0);

    bool chged = false;
    std::vector<Eigen::Vector3d> newPath = converter_[_topic].updateSync(robot_steps_, chged);

    if (chged)
        ROS_INFO("initial path found %i %lu", _topic, newPath.size());

    if (chged)
        publishPath(newPath, _topic);
}

void MultiRobotLocalBehaviorController::publishRobotInfo()
{
    for (uint32_t i = 0; i < robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::RobotInfo ri;
        ri.header.stamp = ros::Time::now();
        ri.header.frame_id = frame_map_;
        ri.robot_name = robot_names_[i];
        ri.pose = robot_pose_[i];
        ri.shape = ri.SHAPE_CIRCLE;
        ri.shape_variables.push_back(robot_radius_[i]);
        ri.sync.robot_id = robot_names_[i];
        ri.sync.current_route_segment = robot_steps_[i];
        ri.mode = ri.MODE_NA;
        ri.status = ri.STATUS_STOPPED; //TODO
        ri.good_id = ri.GOOD_NA;

        pubRobotInfo_.publish(ri);
    }
}
} // namespace tuw_multi_robot_route_to_path
