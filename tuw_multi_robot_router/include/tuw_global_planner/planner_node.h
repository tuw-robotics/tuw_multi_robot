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

#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H


#include <tuw_multi_robot_msgs/PoseIdArray.h>
#include <tuw_multi_robot_msgs/PoseId.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tuw_multi_robot_msgs/VoronoiGraph.h>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>     //DEBUG
#include <tuw_multi_robot_msgs/PlannerStatus.h>
#include <opencv/cv.hpp>
#include <std_msgs/Float32MultiArray.h>

#include <tuw_global_planner/planner.h>
#include <tuw_global_planner/mrr_utils.h>

class Planner_Node :  Planner
{
    public:     Planner_Node(ros::NodeHandle &n);

    public:     void PublishEmpty();
    public:     void Publish(const uint32_t duration);
//ROS:
    public:     ros::NodeHandle                 n_;      ///< Node handler to the root node
    public:     ros::NodeHandle                 n_param_;///< Node handler to the current node
    public:     std::unique_ptr<ros::Rate>      rate_;

// ROS Publishers
    private:
        std::vector<ros::Publisher>         pubPaths_;
        std::vector<ros::Publisher>         pubSegPaths_;
        std::vector<ros::Publisher>         pubVelocityProfile_;
        ros::Publisher                      pubPlannerStatus_;

        std::vector<ros::Subscriber>        subOdom_;
        ros::Subscriber                     subGoalSet_;
        ros::Subscriber                     subMap_;
        ros::Subscriber                     subVoronoiGraph_;


        std::vector<std::string>            robot_names_;
        //private:    std::shared_ptr<unsigned char>      map_;
        cv::Mat                             distMap_;
        Eigen::Vector2d                     mapOrigin_;
        float                               mapResolution_;
        std::vector<float>                  robot_radius_;
        std::string                         segpath_topic_;
        std::string                         odom_topic_;
        std::string                         path_topic_;
        std::string                         goal_topic_;
        std::string                         map_topic_;
        std::string                         voronoi_topic_;
        std::string                         velocity_topic_;
        std::string                         planner_status_topic_;
        bool                                got_map_ = false;
        bool                                got_graph_ = false;
        std::vector<Segment>                graph_;
        size_t                              current_map_hash_;
        size_t                              current_graph_hash_;
        int                                 id_;

        void odomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& _event, int _topic);
        void graphCallback(const tuw_multi_robot_msgs::VoronoiGraph& msg);
        void goalsCallback(const tuw_multi_robot_msgs::PoseIdArray& _goals);
        void mapCallback(const nav_msgs::OccupancyGrid& _map);

        static bool sortSegments(const Segment &i, const Segment &j) {return i.getSegmentId() < j.getSegmentId();}

        size_t getHash(const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution);
        size_t getHash(const std::vector<Segment> &_graph);
};

#endif // PLANNER_NODE_H


