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
#include <opencv/cv.h>
#include <tuw_global_router/point_expander.h>
#include <tuw_global_router/multi_robot_router.h>
#include <tuw_global_router/multi_robot_router_threaded_srr.h>
#include "router.h"

namespace multi_robot_router
{
class DefaultRouter: public multi_robot_router::Router
{
  public:
    explicit DefaultRouter(const uint32_t _nr_robots);
    DefaultRouter();

    void resize(const uint32_t _nr_robots) override ;

    std::unordered_map<std::string, std::vector<Checkpoint>>computePlan(const std::vector<Agent> &agents, const Environment &environment) override;

    bool makePlan(const std::vector<Eigen::Vector3d> &_starts, const std::vector<Eigen::Vector3d> &_goals, const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution, const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph, const std::vector<std::string> &_robot_names) override ;

    void setCollisionResolutionType(const SegmentExpander::CollisionResolverType _cr) override;

    const std::vector<Checkpoint> &getRoute(const uint32_t _robot) const override;

    uint32_t getDuration_ms() const override;

    float getOverallPathLength() const override;

    float getLongestPathLength() const override;

    uint32_t getPriorityScheduleAttemps() const override;

    uint32_t getSpeedScheduleAttemps() const override;

    std::shared_ptr<float> potential_;

    void setPlannerType(routerType _type, uint32_t _nr_threads) override ;

  private:
    //find the right segments to start from
    bool calculateStartPoints(const std::vector<float> &_radius, const cv::Mat &_map, const float &resolution, const Eigen::Vector2d &origin, const std::vector<Segment> &_graph);
    //Find the segment if the graph is a voronoi one
    int32_t getSegment(const std::vector<Segment> &_graph, const Eigen::Vector2d &_odom) const;
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
    bool processEndpointsExpander(const cv::Mat &_map, const std::vector<Segment> &_graph,
                                  const Eigen::Vector2d &_realStart, const Eigen::Vector2d &_realGoal,
                                  Eigen::Vector2d &_voronoiStart, Eigen::Vector2d &_voronoiGoal,
                                  uint32_t &_segmentStart, uint32_t &_segmentGoal, const uint32_t _diameter,
                                  const uint32_t _index) const;

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
    MultiRobotRouter *multiRobotRouter_{};
    MultiRobotRouter mrr_;
    MultiRobotRouterThreadedSrr mrrTs_;
    std::vector<std::vector<Checkpoint>> routingTable_;
    float overallPathLength_{};
    float longestPatLength_{};
    uint32_t duration_{};




};
} // namespace multi_robot_router
#endif // PLANNER_H
