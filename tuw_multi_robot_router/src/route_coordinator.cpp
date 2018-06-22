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

#include <tuw_global_router/route_coordinator.h>

namespace multi_robot_router
{
RouteCoordinatorWrapper::RouteCoordinatorWrapper(const uint32_t _robot, const multi_robot_router::RouteCoordinator &_routeCoordinator)
{
    robot_ = _robot;
    routeCoordinator_ = &_routeCoordinator;
}

bool RouteCoordinatorWrapper::checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, bool _ignoreGoal) const
{
    return routeCoordinator_->checkSegment(_next, _startTime, _endTime, _diameterPixel, _collisionRobot, robot_, _ignoreGoal);
}

int32_t RouteCoordinatorWrapper::findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const
{
    return routeCoordinator_->findPotentialUntilRobotOnSegment(_robot, _segId);
}

const uint32_t RouteCoordinatorWrapper::getEnd() const
{
    return routeCoordinator_->getEnd(robot_);
}

std::vector<std::pair<uint32_t, float>> RouteCoordinatorWrapper::getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const
{
    return routeCoordinator_->getListOfRobotsHigherPrioritizedRobots(_robot, _segId, _potential);
}

const uint32_t RouteCoordinatorWrapper::getStart() const
{
    return routeCoordinator_->getStart(robot_);
}

bool RouteCoordinatorWrapper::isGoal(const Vertex &_seg) const
{
    return routeCoordinator_->isGoal(_seg, robot_);
}

} // namespace multi_robot_router
