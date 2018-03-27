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

#ifndef ROUTE_COORDINATOR_H
#define ROUTE_COORDINATOR_H

#include <tuw_global_planner/mrr_utils.h>
#include <memory>
#include <vector>
#include <tuw_global_planner/srr_utils.h>

namespace multi_robot_router
{
    class RouteCoordinator
    {
        public:
            virtual void reset(const std::vector<Segment>  &_graph, const uint32_t _nrRobots) = 0;
            virtual bool addRoute(const std::vector< RouteVertex > &_path, const uint32_t _diameterPixel, const uint32_t _robotId) = 0;
            virtual bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, const uint32_t _robotId, bool _ignoreGoal = false) const = 0;
            virtual bool setGoalSegments(const std::vector<uint32_t> &_goalSegments) = 0;
            virtual bool setStartSegments(const std::vector<uint32_t> &_startSegments) = 0;
            virtual bool isGoal(const Vertex &_seg, const uint32_t _robotId) const = 0;
            virtual const uint32_t getStart(const uint32_t _robotId) const = 0;
            virtual const uint32_t getEnd(const uint32_t _robotId) const = 0;
            virtual std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const = 0;
            virtual void removeRobot(const uint32_t _robot) = 0;

            virtual int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const = 0; //TODO BETTER VERSION
            virtual int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const = 0;
    };

    class RouteCoordinatorWrapper
    {
        public:
            RouteCoordinatorWrapper(const uint32_t _robot, const RouteCoordinator &_routeCoordinater);
            bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, bool _ignoreGoal = false) const;
            bool isGoal(const Vertex &_seg) const;
            const uint32_t getStart() const;
            const uint32_t getEnd() const;
            std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
            int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const; 
            int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const;
           
        private:
            uint32_t robot_;
            const RouteCoordinator *routeCoordinator_;
    };
}
#endif // PATH_QUERRY_H
