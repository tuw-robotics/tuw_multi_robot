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

#include <tuw_global_router/single_robot_router.h>
#include <tuw_global_router/heuristic.h>
#include <tuw_global_router/potential_calculator.h>

namespace multi_robot_router
{
SingleRobotRouter::SingleRobotRouter() : segment_expander_(SegmentExpander::CollisionResolverType::avoidance)
{
}

SingleRobotRouter::SingleRobotRouter(const SingleRobotRouter &srr) : segment_expander_(srr.getCollisionResolverType())
{
}

void SingleRobotRouter::setCollisionResolver(const SegmentExpander::CollisionResolverType cRes)
{
    segment_expander_.setCollisionResolver(cRes);
}

SegmentExpander::CollisionResolverType SingleRobotRouter::getCollisionResolverType() const
{
    return segment_expander_.getCollisionResolver();
}

bool SingleRobotRouter::getRouteCandidate(const uint32_t _start, const uint32_t _goal, const RouteCoordinatorWrapper &path_coordinator, const uint32_t _robotDiameter, const float &_robotSpeed, std::vector<RouteVertex> &_path, const uint32_t _maxIterations)
{
    robotDiameter_ = _robotDiameter;
    resetAttempt();
    segment_expander_.setSpeed(_robotSpeed);

    if (!segment_expander_.calculatePotentials(&path_coordinator, *(searchGraph_[_start].get()), *(searchGraph_[_goal].get()), _maxIterations, robotDiameter_))
    {
        lastAttempt_ = false;
        return false;
    }

    std::vector<RouteVertex> reversedPath;

    if (!traceback_.getPath(*(searchGraph_[_start].get()), *(searchGraph_[_goal].get()), reversedPath))
    {
        lastAttempt_ = false;
        return false;
    }

    _path.clear();

    for (std::vector<RouteVertex>::const_iterator rit = reversedPath.cend(); rit != reversedPath.cbegin();)
    {
        rit--;
        _path.emplace_back(*rit);
    }

    lastAttempt_ = true;
    return true;
}

void SingleRobotRouter::resetAttempt()
{
    segment_expander_.reset();

    for (std::unique_ptr<Vertex> &v : searchGraph_)
    {
        v->potential = -1;
        v->collision = -1;
        v->weight = 0;
        v->predecessor_ = NULL;
    }
}

bool SingleRobotRouter::getLastResult()
{
    return lastAttempt_;
}

void SingleRobotRouter::initSearchGraph(const std::vector<Segment> &_graph, const uint32_t minSegmentWidth_)
{
    searchGraph_.clear();

    for (const Segment &seg : _graph)
    {
        searchGraph_.push_back(std::make_unique<Vertex>(seg));
    }

    for (std::unique_ptr<Vertex> &v : searchGraph_)
    {
        v->initNeighbours(searchGraph_, minSegmentWidth_);
    }
}

const std::vector<uint32_t> &SingleRobotRouter::getRobotCollisions() const
{
    return segment_expander_.getRobotCollisions();
}
} // namespace multi_robot_router
