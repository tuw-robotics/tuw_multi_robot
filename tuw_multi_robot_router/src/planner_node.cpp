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

#include <tuw_global_planner/planner_node.h>
#include <tuw_global_planner/srr_utils.h>
#include <tuw_multi_robot_msgs/SegmentPath.h>
#include <chrono>
#include <boost/functional/hash.hpp>

//TODO Namespace
//TODO Dynreconf
//TODO Vars
//TODO Graph is Voronoi?
//TODO Astar Dijkstra
//TODO add Weights from robots...
//TODO multithreaded

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tuw_nav_costmap_node");     /// initializes the ros node with default name
    ros::NodeHandle n;

    Planner_Node node(n);
    ros::Rate r(5);

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

//TODO REMOV DEFAULT ROBOTS
Planner_Node::Planner_Node(ros::NodeHandle &_n) :
    Planner(),
    n_(_n),
    n_param_("~"),
    robot_names_(std::vector<std::string> ( {"robot_0", "robot_1", "robot_2"})),    //TODO INIT
             robot_radius_(std::vector<float> ( {0.3, 0.3, 0.3}))
{
    id_ = 0;
    n_param_.param("robot_names", robot_names_, robot_names_);

    std::string robot_names_string = "";
    n_param_.param("robot_names_str", robot_names_string, robot_names_string);

    if(robot_names_string.size() > 0)
    {
        robot_names_string.erase(std::remove(robot_names_string.begin(), robot_names_string.end(), ' '), robot_names_string.end());
        std::istringstream stringStr(robot_names_string);
        std::string result;

        robot_names_.clear();

        while(std::getline(stringStr, result, ','))
        {
            robot_names_.push_back(result);
        }
    }

    resize(robot_names_.size());

    subOdom_.resize(robot_names_.size());
    pubPaths_.resize(robot_names_.size());
    pubSegPaths_.resize(robot_names_.size());
    pubVelocityProfile_.resize(robot_names_.size());

//     useGoalOnSegment_ = false;
//     n_param_.param("use_segment_as_goal", useGoalOnSegment_, useGoalOnSegment_);
//
//     allowEndpointOffSegment_ = true;
//     n_param_.param("allow_endpoint_off_segments", allowEndpointOffSegment_, allowEndpointOffSegment_);
    /*
        optimizationSegmentNr_ = 2;
        n_param_.param("path_optimization_segment_no", optimizationSegmentNr_, optimizationSegmentNr_);*/

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

    segpath_topic_ = "seg_path";
    n_param_.param("seg_path", segpath_topic_, segpath_topic_);

    velocity_topic_ = "vel_profile";
    n_param_.param("velocity_topic_", velocity_topic_, velocity_topic_);

    n_param_.param("robot_radius", robot_radius_, robot_radius_);

    float default_radius = 0.3;
    n_param_.param("robot_default_radius", default_radius, default_radius);

    robot_radius_.resize(robot_names_.size(), default_radius);


    for(size_t i = 0; i < subOdom_.size(); i++)
    {
        subOdom_[i] = _n.subscribe<nav_msgs::Odometry> (robot_names_[i] + "/" + odom_topic_, 1, boost::bind(&Planner_Node::odomCallback, this, _1, i));
        pubPaths_[i] = _n.advertise<nav_msgs::Path> (robot_names_[i] + "/" + path_topic_, 1);
        pubSegPaths_[i] = _n.advertise<tuw_multi_robot_msgs::SegmentPath> (robot_names_[i] + "/" + segpath_topic_, 1);
        pubVelocityProfile_[i] = _n.advertise<std_msgs::Float32MultiArray>(robot_names_[i] + "/" + velocity_topic_, 1);
    }

    subGoalSet_ = _n.subscribe(goal_topic_, 1, &Planner_Node::goalsCallback, this);
    subMap_ = _n.subscribe(map_topic_, 1, &Planner_Node::mapCallback, this);
    subVoronoiGraph_ = _n.subscribe(voronoi_topic_, 1, &Planner_Node::graphCallback, this);

    pubPlannerStatus_ = _n.advertise<tuw_multi_robot_msgs::PlannerStatus> (planner_status_topic_, 1);
}


void Planner_Node::mapCallback(const nav_msgs::OccupancyGrid &_map)
{
    std::vector<signed char> map = _map.data;

    Eigen::Vector2d origin;
    origin[0] = _map.info.origin.position.x;
    origin[1] = _map.info.origin.position.y;

    size_t new_hash = getHash(map, origin, _map.info.resolution);

    ROS_INFO("map %f %f %f", origin[0], origin[1], _map.info.resolution);

    if(new_hash != current_map_hash_)
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

void Planner_Node::odomCallback(const ros::MessageEvent<nav_msgs::Odometry const> &_event, int _robot_nr)
{
    const nav_msgs::Odometry_< std::allocator< void > >::ConstPtr &nav_msg = _event.getMessage();

    Eigen::Vector2d p;
    p[0] = nav_msg->pose.pose.position.x;
    p[1] = nav_msg->pose.pose.position.y;

    updateRobotPose(_robot_nr, p);
}


void Planner_Node::graphCallback(const tuw_multi_robot_msgs::VoronoiGraph &msg)
{
    //TODO Graph Hash
    std::vector<Segment> graph;

    for(const tuw_multi_robot_msgs::Vertex & segment : msg.segments)
    {
        std::vector<Eigen::Vector2d> points;

        for(const geometry_msgs::Point & point : segment.path)
        {
            points.emplace_back(point.x, point.y);
        }

        std::vector<uint32_t> successors;

        for(const auto & succ : segment.successors)
        {
            successors.emplace_back(succ);
        }

        std::vector<uint32_t> predecessors;

        for(const auto & pred : segment.predecessor)
        {
            predecessors.emplace_back(pred);
        }

        graph.emplace_back(segment.id, points, successors, predecessors, segment.minPathSpace);
    }

    std::sort(graph.begin(), graph.end(), sortSegments);

    size_t hash = getHash(graph);

    if(current_graph_hash_ != hash)
    {
        current_graph_hash_ = hash;
        graph_ = graph;
        ROS_INFO("Multi Robot Router: Graph %lu", hash);
    }

////////////////////////////////////////////////////////////
// Debug
////////////////////////////////////////////////////////////
//     for(const Segment &s : graph_)
//     {
//         ROS_INFO("Segment %i", s.getSegmentId());
//         ROS_INFO("\tWidth %f", s.width());
//         ROS_INFO("\tLength %f", s.length());
//         ROS_INFO("\tPredecessors");
//         for(const int &i : s.getPredecessors())
//         {
//           ROS_INFO("\t\t%i", i);
//         }
//         ROS_INFO("\tSuccessors");
//         for(const int &i : s.getSuccessors())
//         {
//           ROS_INFO("\t\t%i", i);
//         }
//
//     }
////////////////////////////////////////////////////////////


    got_graph_ = true;
}


void Planner_Node::goalsCallback(const tuw_multi_robot_msgs::PoseIdArray &_goals)
{
    //Goals have to be orderd
    std::vector<Eigen::Vector2d> goals;

    for(auto it = _goals.poses.begin(); it != _goals.poses.end(); it++)
    {
        Eigen::Vector2d p;
        p[0] = (*it).position.x;
        p[1] = (*it).position.y;
        goals.push_back(p);
    }

    if(got_map_ && got_graph_)
    {
        auto t1 = std::chrono::high_resolution_clock::now();

        if(makePlan(goals, robot_radius_, distMap_, mapResolution_, mapOrigin_, graph_))
        {
            int nx = distMap_.cols;     //TODO Verify
            int ny = distMap_.rows;


            double res = mapResolution_;
            int cx = mapOrigin_[0];
            int cy = mapOrigin_[1];

            auto t2 = std::chrono::high_resolution_clock::now();
            uint32_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            Publish(duration);
            ROS_INFO("Multi Robot Router: Publishing Plan");
        }
        else
        {
            ROS_INFO("Multi Robot Router: No Plan found");

            PublishEmpty();
        }

        auto t2 = std::chrono::high_resolution_clock::now();
        int duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        ROS_INFO("Multi Robot Router: OverallTime %i ms", duration);

        id_++;
    }
}

void Planner_Node::PublishEmpty()
{
    for(int i = 0; i < robot_names_.size(); i++)
    {
        nav_msgs::Path ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubPaths_[i].publish(ros_path);
    }

    for(int i = 0; i < robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::SegmentPath ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";

        pubSegPaths_[i].publish(ros_path);
    }

    tuw_multi_robot_msgs::PlannerStatus ps;
    ps.id = id_;
    ps.success = 0;
    ps.duration = 0;//getDuration_ms();

    pubPlannerStatus_.publish(ps);

}


void Planner_Node::Publish(const uint32_t duration)
{
    for(int i = 0; i < robot_names_.size(); i++)
    {
        nav_msgs::Path ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";
        const std::vector< Checkpoint > &route = getRoute(i);

        //Add first point
        geometry_msgs::PoseStamped p;
        p.header.seq = 0;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";

        Eigen::Vector2d pos = route[0].start * mapResolution_;
        p.pose.position.x = pos[0] + mapOrigin_[0];
        p.pose.position.y = pos[1] + mapOrigin_[1];

        p.pose.orientation.w = 1;
        ros_path.poses.push_back(p);

        //Add other points
        for(const Checkpoint & c : route)
        {
            geometry_msgs::PoseStamped p;
            p.header.seq = 0;
            p.header.stamp = ros::Time::now();
            p.header.frame_id = "map";

            Eigen::Vector2d pos = c.end * mapResolution_;
            p.pose.position.x = pos[0] + mapOrigin_[0];
            p.pose.position.y = pos[1] + mapOrigin_[1];

            p.pose.orientation.w = 1;
            ros_path.poses.push_back(p);
        }

        pubPaths_[i].publish(ros_path);
    }

    for(int i = 0; i < robot_names_.size(); i++)
    {
        tuw_multi_robot_msgs::SegmentPath ros_path;
        ros_path.header.seq = 0;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";
        const std::vector< Checkpoint > &route = getRoute(i);

        for(const Checkpoint & c : route)
        {
            tuw_multi_robot_msgs::PathSegment s;

            Eigen::Vector2d posS = c.start * mapResolution_;

            s.start.x = posS[0] + mapOrigin_[0];
            s.start.y = posS[1] + mapOrigin_[1];

            Eigen::Vector2d posE = c.end * mapResolution_;

            s.end.x = posE[0] + mapOrigin_[0];
            s.end.y = posE[1] + mapOrigin_[1];

            s.segId = c.segId;
            s.width = graph_[c.segId].width() * mapResolution_;   //TODO      (SEG ID WRONG)

            for(int j = 0; j < c.preconditions.size(); j++) //auto precond = (*it).preconditions.cbegin(); precond != (*it).preconditions.cend(); precond++)
            {
                tuw_multi_robot_msgs::PathPrecondition pc;
                pc.robotId = c.preconditions[j].robotId;
                pc.stepCondition = c.preconditions[j].stepCondition;
                s.preconditions.push_back(pc);
            }

            ros_path.poses.push_back(s);
        }

        pubSegPaths_[i].publish(ros_path);
    }


    tuw_multi_robot_msgs::PlannerStatus ps;
    ps.id = id_;
    ps.success = 1;
    //ps.overallPathLength = getOverallPathLength();
    //ps.longestPathLength = getLongestPathLength();
    //ps.prioritySchedulingAttemps = getPriorityScheduleAttemps();
    //ps.speedSchedulingAttemps = getSpeedScheduleAttemps();
    ps.duration = duration;

    pubPlannerStatus_.publish(ps);

}




size_t Planner_Node::getHash(const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution)
{
    std::size_t seed = 0;

    boost::hash_combine(seed, _origin[0]);
    boost::hash_combine(seed, _origin[1]);
    boost::hash_combine(seed, _resolution);

    for(const signed char & val : _map)
    {
        boost::hash_combine(seed, val);
    }

    return seed;
}

std::size_t Planner_Node::getHash(const std::vector< Segment > &_graph)
{
    std::size_t seed = 0;

    for(const Segment & seg : _graph)
    {
        boost::hash_combine(seed, seg.width());
        boost::hash_combine(seed, seg.length());
        boost::hash_combine(seed, seg.getSegmentId());

        for(const int & p : seg.getPredecessors())
        {
            boost::hash_combine(seed, p);
        }

        for(const int & s : seg.getSuccessors())
        {
            boost::hash_combine(seed, s);
        }

        for(const Eigen::Vector2d & vec : seg.getPoints())
        {
            boost::hash_combine(seed, vec[0]);
            boost::hash_combine(seed, vec[1]);
        }
    }

    return seed;
}

