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
#include <tf/tf.h>

//TODO add Weights from robots...

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tuw_multi_robot_router");     /// initializes the ros node with default name
    ros::NodeHandle n;

    multi_robot_router::Planner_Node node(n);
    ros::Rate r(5);

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

namespace multi_robot_router
{
    Planner_Node::Planner_Node(ros::NodeHandle &_n) :
        Planner(),
        n_(_n),
        n_param_("~"),
        robot_names_(std::vector<std::string> ({"robot_0", "robot_1", "robot_2"})),       //{"robot_0", "robot_1", "robot_2"}
        robot_radius_(std::vector<float> ())
    {
        id_ = 0;
        n_param_.param("robot_names", robot_names_, robot_names_);

        //Two possibilities to set robot names (by array in yaml-file or by string (for use in rosrun...))
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

        n_param_.param("robot_radius", robot_radius_, robot_radius_);

        //Robot radius can also be set as string and as array in yaml file
        float default_radius = 0.3;
        n_param_.param("robot_default_radius", default_radius, default_radius);
        robot_radius_.resize(robot_names_.size(), default_radius);


        for(size_t i = 0; i < subOdom_.size(); i++)
        {
            subOdom_[i] = _n.subscribe<nav_msgs::Odometry> (robot_names_[i] + "/" + odom_topic_, 1, boost::bind(&Planner_Node::odomCallback, this, _1, i));
            pubPaths_[i] = _n.advertise<nav_msgs::Path> (robot_names_[i] + "/" + path_topic_, 1);
            pubSegPaths_[i] = _n.advertise<tuw_multi_robot_msgs::SegmentPath> (robot_names_[i] + "/" + segpath_topic_, 1);
        }

        subGoalSet_ = _n.subscribe(goal_topic_, 1, &Planner_Node::goalsCallback, this);
        subMap_ = _n.subscribe(map_topic_, 1, &Planner_Node::mapCallback, this);
        subVoronoiGraph_ = _n.subscribe(voronoi_topic_, 1, &Planner_Node::graphCallback, this);

        pubPlannerStatus_ = _n.advertise<tuw_multi_robot_msgs::PlannerStatus> (planner_status_topic_, 1);
        
        
        call_type = boost::bind(&Planner_Node::parametersCallback, this, _1, _2);
        param_server.setCallback(call_type);
    }

    void Planner_Node::parametersCallback(tuw_multi_robot_router::MultiRobotRouterConfig &config, uint32_t level)
    {
        //Important set router before settings 
        uint32_t threads = config.nr_threads;
        if(config.router_type == 1)
            setPlannerType(routerType::multiThreadSrr, threads);
        else
            setPlannerType(routerType::singleThread, 1);
    
        if(config.collision_resolver == 0)
            setCollisionResolutionType(SegmentExpander::CollisionResolverType::none);
        else if(config.collision_resolver == 1)
            setCollisionResolutionType(SegmentExpander::CollisionResolverType::backtracking);
        else
            setCollisionResolutionType(SegmentExpander::CollisionResolverType::avoidance);
          
        if(config.voronoi_graph)
            graphMode_ = graphType::voronoi;
        else
            graphMode_ = graphType::random;
       
        if(config.goal_mode == 0)
            goalMode_ = goalMode::use_map_goal;
        else if(config.goal_mode == 1)
            goalMode_ = goalMode::use_voronoi_goal;
        else
            goalMode_ = goalMode::use_segment_goal;
        
        routerTimeLimit_s_ = config.router_time_limit_s;
        
        priorityRescheduling_ = config.priority_rescheduling;
        speedRescheduling_ = config.speed_rescheduling;
        segmentOptimizations_ = config.path_endpoint_optimization;
        
          
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
        got_graph_ = true;
    }


    void Planner_Node::goalsCallback(const tuw_multi_robot_msgs::PoseIdArray &_goals)
    {
        //Goals have to be orderd
        std::vector<Eigen::Vector3d> goals;

        for(auto it = _goals.poses.begin(); it != _goals.poses.end(); it++)
        {
            double roll, pitch, yaw;
            Eigen::Vector3d p;
            tf::Quaternion q((*it).orientation.x, (*it).orientation.y, (*it).orientation.z, (*it).orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            ROS_INFO("q %f %f %f %f r %f p %f y %f", q.x(), q.y(), q.z(), q.w(), roll, pitch, yaw);
            
            p[0] = (*it).position.x;
            p[1] = (*it).position.y;     
            p[2] = yaw;
            goals.push_back(p);
        }

        if(got_map_ && got_graph_)
        {
            auto t1 = std::chrono::high_resolution_clock::now();

            if(makePlan(goals, robot_radius_, distMap_, mapResolution_, mapOrigin_, graph_))
            {
                int nx = distMap_.cols;
                int ny = distMap_.rows;


                double res = mapResolution_;
                int cx = mapOrigin_[0];
                int cy = mapOrigin_[1];

                Publish();
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
        ps.duration = getDuration_ms();

        pubPlannerStatus_.publish(ps);

    }


    void Planner_Node::Publish()
    {
        for(int i = 0; i < robot_names_.size(); i++)
        {
            nav_msgs::Path ros_path;
            ros_path.header.seq = 0;
            ros_path.header.stamp = ros::Time::now();
            ros_path.header.frame_id = "map";
            const std::vector< Checkpoint > &route = getRoute(i);

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
            for(const Checkpoint & c : route)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.seq = 0;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";

                Eigen::Vector2d pos(c.end[0] * mapResolution_, c.end[1] * mapResolution_);
                pose.pose.position.x = pos[0] + mapOrigin_[0];
                pose.pose.position.y = pos[1] + mapOrigin_[1];

                tf::Quaternion q;
                q.setEuler(0,0,c.end[2]);
                
                pose.pose.orientation.w = q.w();
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                ros_path.poses.push_back(pose);
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

            for(const Checkpoint & cp : route)
            {
                tuw_multi_robot_msgs::PathSegment seg;

                Eigen::Vector2d posStart(cp.start[0] * mapResolution_, cp.start[1] * mapResolution_);
                tf::Quaternion qStart;
                qStart.setEuler(0,0,cp.start[2]);

                seg.start.position.x = posStart[0] + mapOrigin_[0];
                seg.start.position.y = posStart[1] + mapOrigin_[1];
                seg.start.orientation.w = qStart.w();
                seg.start.orientation.x = qStart.x();
                seg.start.orientation.y = qStart.y();
                seg.start.orientation.z = qStart.z();

                Eigen::Vector2d posEnd(cp.end[0] * mapResolution_, cp.end[1] * mapResolution_);
                tf::Quaternion qEnd;
                qEnd.setEuler(0,0,cp.end[2]);

                seg.end.position.x = posEnd[0] + mapOrigin_[0];
                seg.end.position.y = posEnd[1] + mapOrigin_[1];
                seg.end.orientation.w = qEnd.w();
                seg.end.orientation.x = qEnd.x();
                seg.end.orientation.y = qEnd.y();
                seg.end.orientation.z = qEnd.z();

                seg.segId = cp.segId;
                seg.width = graph_[cp.segId].width() * mapResolution_;

                for(int j = 0; j < cp.preconditions.size(); j++)
                {
                    tuw_multi_robot_msgs::PathPrecondition pc;
                    pc.robotId = cp.preconditions[j].robotId;
                    pc.stepCondition = cp.preconditions[j].stepCondition;
                    seg.preconditions.push_back(pc);
                }

                ros_path.poses.push_back(seg);
            }

            pubSegPaths_[i].publish(ros_path);
        }


        tuw_multi_robot_msgs::PlannerStatus ps;
        ps.id = id_;
        ps.success = 1;
        ps.overallPathLength = (int32_t)getOverallPathLength();
        ps.longestPathLength = (int32_t)getLongestPathLength();
        ps.prioritySchedulingAttemps = (int32_t)getPriorityScheduleAttemps();
        ps.speedSchedulingAttemps = (int32_t)getSpeedScheduleAttemps();
        ps.duration = (int32_t)getDuration_ms();

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
}
