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

//ROS
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_multi_robot_msgs/PlannerStatus.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_multi_robot_router/routerConfig.h>

#include <tuw_global_planner/planner.h>
#include <tuw_global_planner/mrr_utils.h>
#include <opencv/cv.hpp>

//TODO disable got_map if not used

namespace multi_robot_router
{
class Planner_Node : Planner
{
public:
  /**
   * @brief Construct a new Planner_Node object
   * @param n the nodehandle to register publishers and subscribers
   */
  Planner_Node(ros::NodeHandle &n);
  /**
   * @brief publishes an empty RoutingTable 
   */
  void PublishEmpty();
  /**
   * @brief publishes a RoutingTable 
   */
  void Publish();
  /**
   * @brief used to update the nodes timeout to latch topics
   * @param secs the seconds passed since the last update
   */
  void updateTimeout(const float _secs);
  ros::NodeHandle n_;       ///< Node handler to the root node
  ros::NodeHandle n_param_; ///< Node handler to the current node

private:
  class TopicStatus
  {
  public:
    enum class status
    {
      inactive,
      active,
      fixed
    };
    TopicStatus();
    TopicStatus(status _status, const float _activeTime = 1.0);
    void setStatus(status _status, const float _activeTime = 1.0);
    status getStatus() const;
    void updateStatus(const float _updateTime);

  private:
    status status_;
    float activeTime_;
  };

  dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig> param_server;
  dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig>::CallbackType call_type;
  std::vector<ros::Publisher> pubPaths_;
  std::vector<ros::Publisher> pubSegPaths_;
  ros::Publisher pubPlannerStatus_;

  std::vector<ros::Subscriber> subOdom_;
  std::vector<ros::Subscriber> subRobotInfo_;
  ros::Subscriber subGoalSet_;
  ros::Subscriber subMap_;
  ros::Subscriber subVoronoiGraph_;

  std::vector<std::string> subscribed_robot_names_;
  cv::Mat distMap_;
  Eigen::Vector2d mapOrigin_;
  float mapResolution_;
  std::string segpath_topic_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string goal_topic_;
  std::string map_topic_;
  std::string robot_info_topic_;
  std::string voronoi_topic_;
  std::string planner_status_topic_;
  bool got_map_ = false;
  bool got_graph_ = false;
  std::vector<Segment> graph_;
  size_t current_map_hash_;
  size_t current_graph_hash_;
  int id_;
  std::map<std::string, std::pair<TopicStatus, Eigen::Vector3d>> robot_starts_;
  std::map<std::string, std::pair<TopicStatus, Eigen::Vector3d>> robot_goals_;
  std::map<std::string, std::pair<TopicStatus, float>> robot_radius_;
  float topic_timeout_s_ = 10;
  bool freshPlan_ = false;

  void parametersCallback(tuw_multi_robot_router::routerConfig &config, uint32_t level);
  void odomCallback(const ros::MessageEvent<nav_msgs::Odometry const> &_event, int _topic);
  void robotInfoCallback(const ros::MessageEvent<tuw_multi_robot_msgs::RobotInfo const> &_event, int _topic);
  void graphCallback(const tuw_multi_robot_msgs::Graph &msg);
  void goalsCallback(const tuw_multi_robot_msgs::RobotGoalsArray &_goals);
  void mapCallback(const nav_msgs::OccupancyGrid &_map);
  size_t getHash(const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution);
  size_t getHash(const std::vector<Segment> &_graph);
  static bool sortSegments(const Segment &i, const Segment &j) { return i.getSegmentId() < j.getSegmentId(); }
  void unsubscribeTopic(std::string _robot_name);
  void cleanupFixedGoals();
  float getYaw(const geometry_msgs::Quaternion &_rot);
  void tryCreatePlan();
};
} // namespace multi_robot_router
#endif // PLANNER_NODE_H
