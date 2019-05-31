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

#ifndef ROUTER_NODE_H
#define ROUTER_NODE_H

//ROS
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_global_router/robot_info.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_multi_robot_msgs/RouterStatus.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_multi_robot_router/routerConfig.h>

#include <tuw_global_router/mrr_utils.h>
#include <opencv/cv.hpp>

#include <tuw_global_router/router/router.h>
#include <optional>

//TODO disable got_map if not used

namespace multi_robot_router
{
    class RouterNode
    {
    public:
        /**
         * @brief Construct a new Router_Node object
         * @param n the nodehandle to register publishers and subscribers
         */
        explicit RouterNode(Router* router);

        /**
         * Subscribes to topics and initializes publishers.
         * @param n ROS node handle
         */
        void start(ros::NodeHandle& n);

        /**
         * @brief publishes an empty RoutingTable
         */
        void publishEmpty();

        /**
         * @brief publishes a RoutingTable
         */
        void publish();

        /**
         * @brief monitors the execution
         */
        void monitorExecution();

        /**
         * @brief used to update the nodes timeout to latch topics
         * @param secs the seconds passed since the last update
         */
        void updateTimeout(const float _secs);

        ros::NodeHandle n_;       ///< Node handler to the root node
        ros::NodeHandle n_param_; ///< Node handler to the current node

    private:

        Router* router_;

        std::vector<ros::Subscriber> odometry_subscribers_;
        ros::Subscriber goal_subscriber_;
        ros::Subscriber map_subscriber_;
        ros::Subscriber single_goal_subscriber_;
        ros::Subscriber graph_subscriber_;
        ros::Subscriber robot_info_subscriber;

        std::optional<tuw_multi_robot_msgs::Graph> current_graph_;
        std::optional<nav_msgs::OccupancyGrid> current_map_;


        //these 3 members are for time logging
        int attempts_total_;
        int attempts_successful_;
        double sum_processing_time_total_;
        double sum_processing_time_successful_;

        ros::Time time_first_robot_started_;

        tuw_multi_robot_msgs::RouterStatus mrrp_status_;

        dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig> param_server;
        dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig>::CallbackType call_type;
        ros::Publisher pubPlannerStatus_;



        std::vector<RobotInfoPtr> subscribed_robots_;       /// robots avaliable
        std::vector<RobotInfoPtr> active_robots_;           /// robots currently used by the planner
        std::map<std::string, double> finished_robots_;     /// robots finished with execution time
        std::vector<std::string> missing_robots_;
        float robot_radius_max_;
        cv::Mat distMap_;
        Eigen::Vector2d mapOrigin_;
        float mapResolution_;
        bool single_robot_mode_;
        bool publish_routing_table_;
        bool got_map_ = false;
        bool got_graph_ = false;
        std::vector<Segment> graph_;
        size_t current_map_hash_;
        size_t current_graph_hash_;
        int id_;
        float topic_timeout_s_ = 10;
        bool freshPlan_ = false;
        bool monitor_enabled_;

        void parametersCallback(tuw_multi_robot_router::routerConfig &config, uint32_t level);

        void onGraphReceived(const tuw_multi_robot_msgs::Graph& graph);

        void onGoalsReceived(const tuw_multi_robot_msgs::RobotGoalsArray& goals);

        void onMapReceived(const nav_msgs::OccupancyGrid &_map);

        void onRobotInfoReceived(const tuw_multi_robot_msgs::RobotInfo &_robotInfo);

        void onGoalReceived(const geometry_msgs::PoseStamped &_goal);

        size_t getHash(const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution);

        size_t getHash(const std::vector<Segment> &_graph);

        static bool sortSegments(const Segment &i, const Segment &j)
        {
            return i.getSegmentId() < j.getSegmentId();
        }

        float getYaw(const geometry_msgs::Quaternion &_rot);

        bool preparePlanning(std::vector<float> &_radius, std::vector<Eigen::Vector3d> &_starts,
                             std::vector<Eigen::Vector3d> &_goals,
                             const tuw_multi_robot_msgs::RobotGoalsArray &_ros_goals,
                             std::vector<std::string> &robot_names);

        void processGraph(const tuw_multi_robot_msgs::Graph& graph);

        void processRobotInfo(const tuw_multi_robot_msgs::RobotInfo &_robotInfo);

        void processMap(const nav_msgs::OccupancyGrid &map);
    };
}
#endif // Router_Node_H
