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

#ifndef ROUTER_H
#define ROUTER_H

#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_global_router/point_expander.h>
#include <tuw_global_router/multi_robot_router.h>
#include <tuw_global_router/multi_robot_router_threaded_srr.h>

namespace multi_robot_router
{
class Router
{
  public:
    Router(const uint32_t _nr_robots);
    Router();

    /**
            * @brief resizes the planner to a different nr of _nr_robots
            */
    void resize(const uint32_t _nr_robots);
    /**
            * @brief generates the plan from (Vertex[odom robotPose] to Vertex[_goals]
            * @param _starts the start positions of all robots 
            * @param _goals the goal positions fo all robots 
            * @param _radius a vector of the robots radius'
            * @param _map the grid_map used to find the start and goal segments of the path
            * @param _graph the full graph of the map used for planning the path
            * @param _robot_names robot names
            */
    bool makePlan(const std::vector<Eigen::Vector3d> &_starts, const std::vector<Eigen::Vector3d> &_goals, const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution, const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph, const std::vector<std::string> &_robot_names);
    /**
             * @brief sets the CollisionResolverType used 
             */
    void setCollisionResolutionType(const SegmentExpander::CollisionResolverType _cr);
    /**
             * @brief returns the found Routing Table
             * @param _robot the robot to whom the routing table belongs to
             */
    const std::vector<Checkpoint> &getRoute(const uint32_t _robot) const;
    /**
             * @brief getter
             * @returns the duration of the planning attempt 
             */
    uint32_t getDuration_ms() const;
    /**
             * @brief getter
             * @returns the Overall path length of the planning attempt 
             */
    float getOverallPathLength() const;
    /**
             * @brief getter
             * @returns the longest Path length of the planning attempt 
             */
    float getLongestPathLength() const;
    /**
             * @brief getter
             * @returns the priority reschedule attempts
             */
    uint32_t getPriorityScheduleAttemps() const;
    /**
             * @brief getter
             * @returns the speed reschedule attempts
             */
    uint32_t getSpeedScheduleAttemps() const;

    std::shared_ptr<float> potential_;

  private:
    //find the right segments to start from
    bool calculateStartPoints(const std::vector<float> &_radius, const cv::Mat &_map, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph);
    //Find the segment if the graph is a voronoi one
    int32_t getSegment(const std::vector<Segment> &_graph, const Eigen::Vector2d &_pose) const;
    //Helper dist calculation
    float distanceToSegment(const Segment &_s, const Eigen::Vector2d &_p) const;
    //Checks if _seg is a leave of the graph and uses the closes neighbor as segment if the width of the leave is to small
    bool resolveSegment(const std::vector<Segment> &_graph, const uint32_t &_segId, const Eigen::Vector2d &_originPoint, const float &_radius, uint32_t &_foundSeg) const;
    //for every path remove the first and last "segmentOptimizations_" number of segments if possible
    void optimizePaths(const std::vector<Segment> &_graph);
    //adds start and endpoints to the path
    void postprocessRoutingTable();
    //Preprocessing endpoints for enpoint calculation threaded
    bool preprocessEndpoints(const std::vector<float> &_radius, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph);
    //Calculate endpoints
    bool processEndpointsExpander(const cv::Mat &_map, const std::vector<Segment> &_graph, const Eigen::Vector2d &_realStart, const Eigen::Vector2d &_realGoal, Eigen::Vector2d &_voronoiStart, Eigen::Vector2d &_voronoiGoal, uint32_t &_segmentStart, uint32_t &_segmentGoal, const uint32_t _diameter, const uint32_t _index) const;

    uint32_t robot_nr_;
    std::vector<Eigen::Vector3d> starts_;
    std::vector<Eigen::Vector3d> goals_;
    std::vector<Eigen::Vector2d> realGoals_;
    std::vector<Eigen::Vector2d> realStart_;
    std::vector<Eigen::Vector2d> voronoiGoals_;
    std::vector<Eigen::Vector2d> voronoiStart_;
    std::vector<uint32_t> startSegments_;
    std::vector<uint32_t> goalSegments_;
    std::vector<std::string> robot_names_;              /// with the robot id one can access the robot name

    PointExpander pointExpander_;
    MultiRobotRouter *multiRobotRouter_;
    MultiRobotRouter mrr_;
    MultiRobotRouterThreadedSrr mrrTs_;
    std::vector<std::vector<Checkpoint>> routingTable_;
    float overallPathLength_;
    float longestPatLength_;
    uint32_t duration_;

  protected:
    enum class goalMode
    {
        use_segment_goal,
        use_voronoi_goal,
        use_map_goal
    };
    enum class graphType
    {
        voronoi,
        random
    };
    enum class routerType
    {
        singleThread,
        multiThreadSrr
    };
    graphType graphMode_ = graphType::voronoi;
    goalMode goalMode_ = goalMode::use_voronoi_goal;
    float routerTimeLimit_s_ = 10.0;
    bool segmentOptimizations_ = false;
    bool speedRescheduling_ = true;
    bool priorityRescheduling_ = true;
    bool collisionResolver_ = true;

    void setPlannerType(routerType _type, uint32_t _nr_threads);
};
} // namespace multi_robot_router
#endif // PLANNER_H
