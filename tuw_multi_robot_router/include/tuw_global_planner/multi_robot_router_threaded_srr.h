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

#ifndef MRR_THREADED_SRR_H
#define MRR_THREADED_SRR_H

#include <tuw_global_planner/single_robot_router.h>
#include <tuw_global_planner/srr_utils.h>
#include <tuw_global_planner/priority_scheduler.h>
#include <tuw_global_planner/route_coordinator_timed.h>
#include <tuw_global_planner/route_generator.h>
#include <tuw_global_planner/speed_scheduler.h>
#include <tuw_global_planner/segment_expander.h>
#include <tuw_global_planner/multi_robot_router.h>

namespace multi_robot_router
{
    class MultiRobotRouterThreadedSrr : public MultiRobotRouter
    {
        public:
            MultiRobotRouterThreadedSrr(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotRadius, const uint32_t _threads);
            virtual void setRobotNr(const uint32_t _nr_robots);
            virtual void setRobotRadius(const std::vector<uint32_t> &_radius);
            virtual bool getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector<Checkpoint>> &_routingTable, const float &_timeLimit);
            virtual const uint32_t getPriorityScheduleAttempts() const;
            virtual const uint32_t getSpeedScheduleAttempts() const;
            virtual void setCollisionResolver(const SegmentExpander::CollisionResolverType cRes);
            virtual void setPriorityRescheduling(const bool _status);
            virtual void setSpeedRescheduling(const bool _status);
        private:
            void resetAttempt(const std::vector< Segment > &_graph);
            bool planPaths(const std::vector<uint32_t> &_priorityList, const std::vector<float> &_speedList, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, const uint32_t _firstSchedule, std::vector<std::vector<RouteVertex>> &_routeCandidates, uint32_t &_robot);
            PriorityScheduler priority_scheduler_;
            SpeedScheduler speed_scheduler_;
            std::unique_ptr<RouteCoordinator> route_coordinator_;
            bool useSpeedRescheduler_ = true;
            bool usePriorityRescheduler_ = true;

            uint32_t nr_robots_;
            uint32_t min_diameter_;
            std::vector<uint32_t> robotDiameter_;
            std::vector<std::vector<uint32_t>> robotCollisions_;
            uint32_t priorityScheduleAttempts_;
            uint32_t speedScheduleAttempts_;
            SegmentExpander::CollisionResolverType cResType_;
            uint32_t maxIterationsSingleRobot_;
            uint32_t threads_ = 10;
            std::vector<SingleRobotRouter> srr;

    };
}
#endif
