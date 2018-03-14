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

#include <tuw_global_planner/multi_robot_router.h>
#include <tuw_global_planner/single_robot_router.h>
#include <iostream>
#include <ros/ros.h>

MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotRadius) : RouteGenerator()
{
    priority_scheduler_ = std::make_unique<PriorityScheduler> (_nr_robots);
    route_coordinator_ = std::make_unique<RouteCoordinatorTimed> ();
    setRobotNr(_nr_robots);
    robotDiameter_ = _robotRadius;
}

void MultiRobotRouter::setRobotNr(const uint32_t _nr_robots)
{
    nr_robots_ = _nr_robots;
    priority_scheduler_->reset(_nr_robots);
}

void MultiRobotRouter::setRobotRadius(const std::vector< uint32_t > &_diameter)
{
    robotDiameter_.clear();
    robotDiameter_ = _diameter;
}

void MultiRobotRouter::resetAttempt(const std::vector< Segment > &_graph)
{
    route_coordinator_->reset(_graph, nr_robots_);
    priority_scheduler_->reset(nr_robots_);
    robotCollisions_.clear();
    robotCollisions_.resize(nr_robots_);
}

bool MultiRobotRouter::getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &startSegments, const std::vector<uint32_t> &goalSegments, std::vector<std::vector< Checkpoint>> &routingTable)
{
    resetAttempt(_graph);
    route_coordinator_->setStartSegments(startSegments);
    route_coordinator_->setGoalSegments(goalSegments);

    //TODO Priority Rescheduling
    //TODO Speed Rescheduling

    SingleRobotRouter srr;
    srr.initSearchGraph(_graph);

    std::vector<std::vector<RouteVertex>> routeCandidates;
    routeCandidates.resize(nr_robots_);

    for(int i = 0; i < nr_robots_; i++)
    {
        route_coordinator_->setActive(i);

        if(!srr.getRouteCandidate(startSegments[i], goalSegments[i], *route_coordinator_.get(), robotDiameter_[i], routeCandidates[i]))
            return false;

        robotCollisions_[i] = srr.getRobotCollisions();

        if(!route_coordinator_->addRoute(routeCandidates[i], robotDiameter_[i]))
            return false;

        //DEBUG
//         ROS_INFO("Route Candidate");
//         for(const RouteVertex & rv : routeCandidates[i])
//         {
//             ROS_INFO("Vertex %i: pot: %i", rv.getSegment().getSegmentId(), rv.potential);
//         }
//
//         ROS_INFO("Collisions");
//         for(const uint32_t & vec : robotCollisions_[i])
//         {
//             ROS_INFO("Coll %u", vec);
//         }
    }

    routingTable = generatePath(routeCandidates, *(route_coordinator_.get()));

    return true;
}

