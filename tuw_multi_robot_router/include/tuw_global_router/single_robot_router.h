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

#ifndef SRR_H
#define SRR_H

#include <tuw_global_router/srr_utils.h>
#include <tuw_global_router/route_coordinator.h>
#include <tuw_global_router/segment_expander.h>
#include <tuw_global_router/traceback.h>
#include <iostream>

namespace multi_robot_router
{
typedef struct Robot
{
    int id;
    float diameter;
    float speedMultiplier;
    Robot(int _id, float _d, float _s) : id(_id), diameter(_d), speedMultiplier(_s) {}
    Robot(int _id, float _d) : Robot(_id, _d, 1) {}
} Robot;

class SingleRobotRouter
{
  public:
    /** 
             * @brief constructor 
             */
    SingleRobotRouter();
    /** 
             * @brief copy constructor 
             */
    SingleRobotRouter(const SingleRobotRouter &srr);
    /**
             * @brief calculates a route candidate coordinated with other robots by the given route Coordinated
             * @param _start the start vertex
             * @param _goal the goal vertex
             * @param path_coordinator the used route coordinater to coordinate with other robots
             * @param _robotDiameter the diameter of the robot
             * @param _robotSpeed the selected robot speed (multiplier)
             * @param path the found route candidate
             * @param _maxIterations the maximum allowed iterations of the loop
             * @returns returns true if a route candidate is found
             */
    bool getRouteCandidate(const uint32_t _start, const uint32_t _goal, const RouteCoordinatorWrapper &path_coordinator, const uint32_t _robotDiameter, const float &_robotSpeed, std::vector<RouteVertex> &path, const uint32_t _maxIterations);

    /**
             * @brief returns the found robot collisions while planning
             * @returns the found robot collisions 
             */
    const std::vector<uint32_t> &getRobotCollisions() const;
    /**
             * @brief generates the search graph out of the given graph (and optimizes it)
             * @param _graph the main graph
             * @param minSegmentWidth_ used to optimize the graph (e.g. if no robot has less than d in size all segments with less space than d can be removed) 
             */
    void initSearchGraph(const std::vector<Segment> &_graph, const uint32_t minSegmentWidth_ = 0);
    /**
             * @brief sets the collisionResolver used 
             */
    void setCollisionResolver(const SegmentExpander::CollisionResolverType cRes);
    /**
             * @brief returns the result of the last planning attempt 
             */
    bool getLastResult();
    /**
             * @brief returns the CollisionResolverType 
             */
    SegmentExpander::CollisionResolverType getCollisionResolverType() const;

  private:
    void resetAttempt();
    SegmentExpander segment_expander_;
    Traceback traceback_;
    uint32_t robotDiameter_;
    //unique_ptr to keep references of Vertex (Heap), because the list is updated while runtime
    std::vector<std::unique_ptr<Vertex>> searchGraph_;
    bool lastAttempt_ = false;
    std::vector<RouteVertex> path_;
};
} // namespace multi_robot_router
#endif
