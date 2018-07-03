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

#include <tuw_global_router/router_node.h>
#include <tuw_global_router/srr_utils.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <tf/tf.h>

//TODO add Weights from robots...

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tuw_multi_robot_router"); /// initializes the ros node with default name
    ros::NodeHandle n;

    ros::Rate r(5);

    multi_robot_router::Router_Node node(n);

    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        node.updateTimeout(r.expectedCycleTime().toSec());
    }

    return 0;
}

namespace multi_robot_router
{
Router_Node::Router_Node(ros::NodeHandle &_n) : Router(),
                                                n_(_n),
                                                n_param_("~")
{
    id_ = 0;

    planner_status_topic_ = "planner_status";
    n_param_.param("planner_status_topic", planner_status_topic_, planner_status_topic_);

    odom_topic_ = "odom";
    n_param_.param("odom_topic", odom_topic_, odom_topic_);

    path_topic_ = "path_unsynced";
    n_param_.param("path_topic", path_topic_, path_topic_);

    goal_topic_ = "goals";
    n_param_.param("goal_topic", goal_topic_, goal_topic_);

    map_topic_ = "/map";
    n_param_.param("map_topic", map_topic_, map_topic_);

    voronoi_topic_ = "segments";
    n_param_.param("graph_topic", voronoi_topic_, voronoi_topic_);

    route_topic_ = "route";
    n_param_.param("route", route_topic_, route_topic_);

    robot_info_topic_ = "/robot_info";
    n_param_.param("robot_info", robot_info_topic_, robot_info_topic_);

    singleRobotName_ = "";
    n_param_.param("robot_name", singleRobotName_, singleRobotName_);

    singleRobotGoalTopic_ = "/goal";
    n_param_.param("robot_goal", singleRobotGoalTopic_, singleRobotGoalTopic_);

    // static subscriptions
    subGoalSet_ = _n.subscribe(goal_topic_, 1, &Router_Node::goalsCallback, this);
    subMap_ = _n.subscribe(map_topic_, 1, &Router_Node::mapCallback, this);
    subVoronoiGraph_ = _n.subscribe(voronoi_topic_, 1, &Router_Node::graphCallback, this);
    subRobotInfo_ = _n.subscribe(robot_info_topic_, 1000, &Router_Node::robotInfoCallback, this);

    if (!singleRobotName_.size() == 0)
    {
        subSingleRobotGoal_ = _n.subscribe(singleRobotGoalTopic_, 1, &Router_Node::goalCallback, this);
    }

    //static publishers
    pubPlannerStatus_ = _n.advertise<tuw_multi_robot_msgs::RouterStatus>(planner_status_topic_, 1);

    //dynamic reconfigure
    call_type = boost::bind(&Router_Node::parametersCallback, this, _1, _2);
    param_server.setCallback(call_type);
}

void Router_Node::goalCallback(const geometry_msgs::PoseStamped &_goal)
{
    tuw_multi_robot_msgs::RobotGoals goal;
    goal.robot_name = singleRobotName_;
    goal.path_points.push_back(_goal.pose);

    tuw_multi_robot_msgs::RobotGoalsArray goals;
    goals.goals.push_back(goal);

    goalsCallback(goals);
}

void Router_Node::updateTimeout(const float _secs)
{
    //Todo update timeouts and clear old messages
    for (auto it = robot_starts_.begin(); it != robot_starts_.end(); it++)
    {
        (*it).second.first.updateStatus(_secs);
    }

    for (auto it = robot_radius_.begin(); it != robot_radius_.end(); it++)
    {
        (*it).second.first.updateStatus(_secs);
    }
}
void Router_Node::parametersCallback(tuw_multi_robot_router::routerConfig &config, uint32_t level)
{
    //Important set router before settings
    uint32_t threads = config.nr_threads;
    if (config.router_type == 1)
        setPlannerType(routerType::multiThreadSrr, threads);
    else
        setPlannerType(routerType::singleThread, 1);

    if (config.collision_resolver == 0)
        setCollisionResolutionType(SegmentExpander::CollisionResolverType::none);
    else if (config.collision_resolver == 1)
        setCollisionResolutionType(SegmentExpander::CollisionResolverType::backtracking);
    else
        setCollisionResolutionType(SegmentExpander::CollisionResolverType::avoidance);

    if (config.voronoi_graph)
        graphMode_ = graphType::voronoi;
    else
        graphMode_ = graphType::random;

    if (config.goal_mode == 0)
        goalMode_ = goalMode::use_map_goal;
    else if (config.goal_mode == 1)
        goalMode_ = goalMode::use_voronoi_goal;
    else
        goalMode_ = goalMode::use_segment_goal;

    routerTimeLimit_s_ = config.router_time_limit_s;
    topic_timeout_s_ = config.topic_timeout_s;

    priorityRescheduling_ = config.priority_rescheduling;
    speedRescheduling_ = config.speed_rescheduling;
    segmentOptimizations_ = config.path_endpoint_optimization;
}

void Router_Node::mapCallback(const nav_msgs::OccupancyGrid &_map)
{
    std::vector<signed char> map = _map.data;

    Eigen::Vector2d origin;
    origin[0] = _map.info.origin.position.x;
    origin[1] = _map.info.origin.position.y;

    size_t new_hash = getHash(map, origin, _map.info.resolution);

    ROS_INFO("map %f %f %f", origin[0], origin[1], _map.info.resolution);

    if (new_hash != current_map_hash_)
    {
        mapOrigin_[0] = _map.info.origin.position.x;
        mapOrigin_[1] = _map.info.origin.position.y;
        mapResolution_ = _map.info.resolution;

        cv::Mat m(_map.info.height, _map.info.width, CV_8SC1, map.data());

        m.convertTo(m, CV_8UC1);
        cv::bitwise_not(m, m);

        cv::threshold(m, m, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        cv::distanceTransform(m, distMap_, CV_DIST_L1, 3);

        current_map_hash_ = new_hash;
        got_map_ = true;

        ROS_INFO("Multi Robot Router: New Map %i %i %lu", _map.info.width, _map.info.height, current_map_hash_);
    }
}

void Router_Node::odomCallback(const ros::MessageEvent<nav_msgs::Odometry const> &_event, int _robot_nr)
{
    if (robot_starts_[subscribed_robot_names_[_robot_nr]].first.getStatus() == TopicStatus::status::fixed) //Don't update fixed poses
        return;

    const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &nav_msg = _event.getMessage();
    Eigen::Vector3d start(nav_msg->pose.pose.position.x, nav_msg->pose.pose.position.y, getYaw(nav_msg->pose.pose.orientation));
    TopicStatus s(TopicStatus::status::active, topic_timeout_s_);
    std::pair<TopicStatus, Eigen::Vector3d> start_pair(s, start);

    if (robot_starts_.find(subscribed_robot_names_[_robot_nr]) == robot_starts_.end())
    {
        robot_starts_.emplace(subscribed_robot_names_[_robot_nr], start_pair);
    }
    else
    {
        robot_starts_[subscribed_robot_names_[_robot_nr]] = start_pair;
    }
}

float Router_Node::calcRadius(const int shape, const std::vector<float> &shape_variables) const
{
    tuw_multi_robot_msgs::RobotInfo ri;
    if (shape == ri.SHAPE_CIRCLE)
    {
        return shape_variables[0];
    }

    return -1;
}

void Router_Node::robotInfoCallback(const tuw_multi_robot_msgs::RobotInfo &_robotInfo)
{
    TopicStatus s(TopicStatus::status::active, topic_timeout_s_);
    std::pair<TopicStatus, float> radius_pair(s, calcRadius(_robotInfo.shape, _robotInfo.shape_variables));

    if (std::find(subscribed_robot_names_.begin(), subscribed_robot_names_.end(), _robotInfo.robot_name) == subscribed_robot_names_.end())
    {
        subscribed_robot_names_.push_back(_robotInfo.robot_name);
        robot_radius_.emplace(_robotInfo.robot_name, radius_pair);
        //Not existant subscribe robots
        ROS_INFO("Multi Robot Router: subscribing to %s", (_robotInfo.robot_name + "/" + odom_topic_).c_str());
        subOdom_.emplace_back(n_.subscribe<nav_msgs::Odometry>(_robotInfo.robot_name + "/" + odom_topic_, 1, boost::bind(&Router_Node::odomCallback, this, _1, subscribed_robot_names_.size() - 1)));

        ROS_INFO("Multi Robot Router: advertising on %s", (_robotInfo.robot_name + "/" + path_topic_).c_str());
        pubPaths_.emplace_back(n_.advertise<nav_msgs::Path>(_robotInfo.robot_name + "/" + path_topic_, 1, true));
        ROS_INFO("Multi Robot Router: advertising on %s", (_robotInfo.robot_name + "/" + route_topic_).c_str());
        pubSegPaths_.emplace_back(n_.advertise<tuw_multi_robot_msgs::Route>(_robotInfo.robot_name + "/" + route_topic_, 1, true));
    }
    else
    {
        robot_radius_[_robotInfo.robot_name] = radius_pair;
    }
    
    robot_radius_max_ = 0;
    for(auto&& it = robot_radius_.begin(); it != robot_radius_.end(); it++)
    {
      if(it->second.second > robot_radius_max_)
        robot_radius_max_ = it->second.second;
    }
}

void Router_Node::graphCallback(const tuw_multi_robot_msgs::Graph &msg)
{
    std::vector<Segment> graph;

    for (const tuw_multi_robot_msgs::Vertex &segment : msg.vertices)
    {
        std::vector<Eigen::Vector2d> points;

        for (const geometry_msgs::Point &point : segment.path)
        {
            points.emplace_back(point.x, point.y);
        }

        std::vector<uint32_t> successors;

        for (const auto &succ : segment.successors)
        {
            successors.emplace_back(succ);
        }

        std::vector<uint32_t> predecessors;

        for (const auto &pred : segment.predecessors)
        {
            predecessors.emplace_back(pred);
        }

        if (segment.valid)
        {
            graph.emplace_back(segment.id, points, successors, predecessors,  3 * robot_radius_max_ / mapResolution_); //segment.width);
        }
        else
        {
            graph.emplace_back(segment.id, points, successors, predecessors, 0);
        }
    }

    std::sort(graph.begin(), graph.end(), sortSegments);

    size_t hash = getHash(graph);

    if (current_graph_hash_ != hash)
    {
        current_graph_hash_ = hash;
        graph_ = graph;
        ROS_INFO("Multi Robot Router: Graph %lu", hash);
    }
    got_graph_ = true;
}

bool Router_Node::preparePlanning(std::vector<float> &_radius, std::vector<Eigen::Vector3d> &_starts, std::vector<Eigen::Vector3d> &_goals, const tuw_multi_robot_msgs::RobotGoalsArray &_rosGoals)
{
    bool retval = true;
    missing_robots_.clear();
    std::vector<std::string> robot_names;
    for (int i = 0; i < _rosGoals.goals.size(); i++)
    {
        std::string name = _rosGoals.goals[i].robot_name;
        //Check duplicated goals
        if (std::find(robot_names.begin(), robot_names.end(), name) != robot_names.end())
        {
            ROS_INFO("Multi Robot Router: Too many goals for one robot");
            publishEmpty();
            return false;
        }
        robot_names.push_back(name);

        //Check if a robot is existant in general
        if (robot_radius_[name].first.getStatus() == TopicStatus::status::active)
        {
            //Check if robot topics are active and save their values
            std::pair<TopicStatus, float> radius_pair = robot_radius_[name];
            _radius.push_back(robot_radius_[name].second);

            if (_rosGoals.goals[i].path_points.size() == 0)
            {
                ROS_INFO("Multi Robot Router: To less goal points for robot %s", name.c_str());
                return false;
            }
            Eigen::Vector3d goal(_rosGoals.goals[i].path_points.back().position.x, _rosGoals.goals[i].path_points.back().position.y, getYaw(_rosGoals.goals[i].path_points.back().orientation));
            _goals.push_back(goal);
        }
        else
        {
            ROS_INFO("Multi Robot Router: Inactive Robot (%s)", name.c_str());
            retval = false;
            missing_robots_.push_back(name);
        }

        if (_rosGoals.goals[i].path_points.size() == 1)
        {
            //use current robot pose
            if (robot_starts_[name].first.getStatus() == TopicStatus::status::active)
            {
                _starts.push_back(robot_starts_[name].second);
            }
            else
            {
                ROS_INFO("Multi Robot Router: Inactive Robot (%s)", name.c_str());
                retval = false;
                missing_robots_.push_back(name);
            }
        }
        else
        {
            Eigen::Vector3d start(_rosGoals.goals[i].path_points.front().position.x, _rosGoals.goals[i].path_points.front().position.y, getYaw(_rosGoals.goals[i].path_points.front().orientation));
            _starts.push_back(start);
        }
    }

    return retval;
}

void Router_Node::goalsCallback(const tuw_multi_robot_msgs::RobotGoalsArray &_goals)
{
    //Get robots
    std::vector<Eigen::Vector3d> starts;
    std::vector<Eigen::Vector3d> goals;
    std::vector<float> radius;

    bool preparationSuccessful = preparePlanning(radius, starts, goals, _goals);

    if (preparationSuccessful && got_map_ && got_graph_)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        preparationSuccessful &= makePlan(starts, goals, radius, distMap_, mapResolution_, mapOrigin_, graph_);
        if (preparationSuccessful)
        {
            int nx = distMap_.cols;
            int ny = distMap_.rows;

            double res = mapResolution_;
            int cx = mapOrigin_[0];
            int cy = mapOrigin_[1];

            publish();
            ROS_INFO("Multi Robot Router: Publishing Plan");
            freshPlan_ = false;
        }
        else
        {
            ROS_INFO("Multi Robot Router: No Plan found");

            publishEmpty();
        }

        auto t2 = std::chrono::high_resolution_clock::now();
        int duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        ROS_INFO("Multi Robot Router: OverallTime %i ms", duration);

        id_++;
    }
    else if (!got_map_ || !got_graph_)
    {
        publishEmpty();
        ROS_INFO("Multi Robot Router: No Map or Graph received");
    }
    else
    {
        publishEmpty();
    }
}

float Router_Node::getYaw(const geometry_msgs::Quaternion &_rot)
{
    double roll, pitch, yaw;

    tf::Quaternion q(_rot.x, _rot.y, _rot.z, _rot.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

void Router_Node::publishEmpty()
{
    for (int i = 0; i < subscribed_robot_names_.size(); i++)
    {
        nav_msgs::Path ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubPaths_[i].publish(ros_path);
    }

    for (int i = 0; i < subscribed_robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::Route ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubSegPaths_[i].publish(ros_path);
    }

    tuw_multi_robot_msgs::RouterStatus ps;
    ps.id = id_;
    ps.success = 0;
    ps.duration = getDuration_ms();
    for (const std::string &name : missing_robots_)
    {
        ps.missing_robots.push_back(name);
    }

    pubPlannerStatus_.publish(ps);
}

void Router_Node::publish()
{
    for (int i = 0; i < subscribed_robot_names_.size(); i++)
    {
        nav_msgs::Path ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";
        const std::vector<Checkpoint> &route = getRoute(i);

        //Add first point
        geometry_msgs::PoseStamped pose_1;
        pose_1.header.seq = 0;
        pose_1.header.stamp = ros::Time::now();
        pose_1.header.frame_id = "map";

        Eigen::Vector2d pos(route[0].start[0] * mapResolution_, route[0].start[1] * mapResolution_);
        pose_1.pose.position.x = pos[0] + mapOrigin_[0];
        pose_1.pose.position.y = pos[1] + mapOrigin_[1];

        pose_1.pose.orientation.w = 1;
        ros_path.poses.push_back(pose_1);

        //Add other points
        for (const Checkpoint &c : route)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.seq = 0;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";

            Eigen::Vector2d pos(c.end[0] * mapResolution_, c.end[1] * mapResolution_);
            pose.pose.position.x = pos[0] + mapOrigin_[0];
            pose.pose.position.y = pos[1] + mapOrigin_[1];

            tf::Quaternion q;
            q.setEuler(0, 0, c.end[2]);

            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            ros_path.poses.push_back(pose);
        }

        pubPaths_[i].publish(ros_path);
    }

    for (int i = 0; i < subscribed_robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::Route ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";
        const std::vector<Checkpoint> &route = getRoute(i);

        for (const Checkpoint &cp : route)
        {
            tuw_multi_robot_msgs::RouteSegment seg;

            Eigen::Vector2d posStart(cp.start[0] * mapResolution_, cp.start[1] * mapResolution_);
            tf::Quaternion qStart;
            qStart.setEuler(0, 0, cp.start[2]);

            seg.start.position.x = posStart[0] + mapOrigin_[0];
            seg.start.position.y = posStart[1] + mapOrigin_[1];
            seg.start.orientation.w = qStart.w();
            seg.start.orientation.x = qStart.x();
            seg.start.orientation.y = qStart.y();
            seg.start.orientation.z = qStart.z();

            Eigen::Vector2d posEnd(cp.end[0] * mapResolution_, cp.end[1] * mapResolution_);
            tf::Quaternion qEnd;
            qEnd.setEuler(0, 0, cp.end[2]);

            seg.end.position.x = posEnd[0] + mapOrigin_[0];
            seg.end.position.y = posEnd[1] + mapOrigin_[1];
            seg.end.orientation.w = qEnd.w();
            seg.end.orientation.x = qEnd.x();
            seg.end.orientation.y = qEnd.y();
            seg.end.orientation.z = qEnd.z();

            seg.segment_id = cp.segId;
            seg.width = graph_[cp.segId].width() * mapResolution_;

            for (int j = 0; j < cp.preconditions.size(); j++)
            {
                tuw_multi_robot_msgs::RoutePrecondition pc;
                pc.robot_id = subscribed_robot_names_[cp.preconditions[j].robotId];
                pc.current_route_segment = cp.preconditions[j].stepCondition;
                seg.preconditions.push_back(pc);
            }

            ros_path.segments.push_back(seg);
        }

        pubSegPaths_[i].publish(ros_path);
    }

    tuw_multi_robot_msgs::RouterStatus ps;
    ps.id = id_;
    ps.success = 1;
    ps.overall_path_length = (int32_t)getOverallPathLength();
    ps.longest_path_length = (int32_t)getLongestPathLength();
    ps.priority_scheduling_attemps = (int32_t)getPriorityScheduleAttemps();
    ps.speed_scheduling_attemps = (int32_t)getSpeedScheduleAttemps();
    ps.duration = (int32_t)getDuration_ms();

    pubPlannerStatus_.publish(ps);
}

size_t Router_Node::getHash(const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution)
{
    std::size_t seed = 0;

    boost::hash_combine(seed, _origin[0]);
    boost::hash_combine(seed, _origin[1]);
    boost::hash_combine(seed, _resolution);

    for (const signed char &val : _map)
    {
        boost::hash_combine(seed, val);
    }

    return seed;
}

std::size_t Router_Node::getHash(const std::vector<Segment> &_graph)
{
    std::size_t seed = 0;

    for (const Segment &seg : _graph)
    {
        boost::hash_combine(seed, seg.width());
        boost::hash_combine(seed, seg.length());
        boost::hash_combine(seed, seg.getSegmentId());

        for (const int &p : seg.getPredecessors())
        {
            boost::hash_combine(seed, p);
        }

        for (const int &s : seg.getSuccessors())
        {
            boost::hash_combine(seed, s);
        }

        for (const Eigen::Vector2d &vec : seg.getPoints())
        {
            boost::hash_combine(seed, vec[0]);
            boost::hash_combine(seed, vec[1]);
        }
    }

    return seed;
}

Router_Node::TopicStatus::TopicStatus(const status _status, const float _activeTime)
{
    setStatus(_status, _activeTime);
}

Router_Node::TopicStatus::TopicStatus() : TopicStatus(status::inactive)
{
}

Router_Node::TopicStatus::status Router_Node::TopicStatus::getStatus() const
{
    return status_;
}

void Router_Node::TopicStatus::updateStatus(const float _updateTime)
{
    if (activeTime_ > 0)
        activeTime_ -= _updateTime;

    if (activeTime_ < 0)
        activeTime_ = 0;

    if (activeTime_ == 0 && status_ != status::fixed)
        status_ = status::inactive;
}

void Router_Node::TopicStatus::setStatus(const status _status, const float _activeTime)
{
    if (_status != status::active)
        activeTime_ = 0;
    else
        activeTime_ = _activeTime;

    status_ = _status;
}

} // namespace multi_robot_router
