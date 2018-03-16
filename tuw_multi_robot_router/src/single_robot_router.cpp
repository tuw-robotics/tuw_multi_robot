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

#include <tuw_global_planner/single_robot_router.h>
#include <tuw_global_planner/heuristic.h>
#include <tuw_global_planner/potential_calculator.h>

SingleRobotRouter::SingleRobotRouter()
{
    //Used Heuristic and PotentialCalculator as input to be able to switch them at runtime (for future use :D)
    Heuristic h;
    PotentialCalculator p;
    segment_expander_ = std::make_unique<SegmentExpander>(h, p);
    traceback_ = std::make_unique<Traceback>();
}

bool SingleRobotRouter::getRouteCandidate(const uint32_t _start, const uint32_t _goal, const RouteCoordinator &path_coordinator, const uint32_t _radius, std::vector<RouteVertex> &_path)
{
    radius_ = _radius;
    resetAttempt();

    if(!segment_expander_->calculatePotentials(&path_coordinator, *(searchGraph_[_start].get()), *(searchGraph_[_goal].get()), searchGraph_.size() * 20, radius_)) //TODO *LIGHTNING* no fixed numbers :D
        return false;

    std::vector<RouteVertex> reversedPath;

    if(!traceback_->getPath(*(searchGraph_[_start].get()), *(searchGraph_[_goal].get()), reversedPath))
        return false;

    _path.clear();

    for(std::vector<RouteVertex>::const_iterator rit = reversedPath.cend(); rit != reversedPath.cbegin();)
    {
        rit--;
        _path.emplace_back(*rit);
    }

    return true;
}

void SingleRobotRouter::resetAttempt()
{
    segment_expander_->reset();

    for(std::unique_ptr<Vertex> & v : searchGraph_)
    {
        v->potential = -1;
        v->collision = -1;
        v->weight = 0;
        v->predecessor_ = NULL;
    }
}

void SingleRobotRouter::initSearchGraph(const std::vector< Segment > &_graph, const uint32_t minSegmentWidth_)
{
    searchGraph_.clear();

    for(const Segment & seg : _graph)
    {
        searchGraph_.push_back(std::make_unique<Vertex>(seg));
    }

    for(std::unique_ptr<Vertex> & v : searchGraph_)
    {
        v->initNeighbours(searchGraph_, minSegmentWidth_);
    }
}

const std::vector<uint32_t> &SingleRobotRouter::getRobotCollisions() const
{
    return segment_expander_->getRobotCollisions();
}
