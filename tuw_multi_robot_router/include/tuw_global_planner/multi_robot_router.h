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

#ifndef MRR_H
#define MRR_H

#include <tuw_global_planner/srr_utils.h>
#include <tuw_global_planner/priority_scheduler.h>
#include <tuw_global_planner/route_coordinator_timed.h>
#include <tuw_global_planner/route_generator.h>

class MultiRobotRouter : RouteGenerator
{
    public:
        MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotRadius);
        void setRobotNr(const uint32_t _nr_robots);
        void setRobotRadius(const std::vector<uint32_t> &_radius);
        bool getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector<Checkpoint>>& _routingTable);
    private:
        void resetAttempt(const std::vector< Segment > &_graph);
        std::unique_ptr<PriorityScheduler> priority_scheduler_;
        std::unique_ptr<RouteCoordinator> route_coordinator_;
        
        int nr_robots_;
        std::vector<uint32_t> robotDiameter_;
        std::vector<std::vector<uint32_t>> robotCollisions_;
        
};

#endif
