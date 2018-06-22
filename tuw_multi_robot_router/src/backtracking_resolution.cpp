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

#include <tuw_global_router/backtracking_resolution.h>
#include <iostream>
#define TIMEOVERLAP (1)

namespace multi_robot_router
{
BacktrackingResolution::BacktrackingResolution(uint32_t _timeoverlap) : timeoverlap_(_timeoverlap)
{
}

BacktrackingResolution::BacktrackingResolution() : BacktrackingResolution(TIMEOVERLAP)
{
}

void BacktrackingResolution::resetSession(const RouteCoordinatorWrapper *_route_querry, const PotentialCalculator *_pCalc, const uint32_t _robotDiameter)
{
    route_querry_ = _route_querry;
    pCalc_ = _pCalc;
    robotDiameter_ = _robotDiameter;
    generatedSubgraphs_.clear();
    encounteredCollisions_.clear();
    resolutionAttemp_ = 0;
    avoidStartPredecessorDone_ = false;
    avoidStartSuccessorDone_ = false;
}

void BacktrackingResolution::addCollision(const uint32_t robot)
{
    if (encounteredCollisions_.size() <= robot)
        encounteredCollisions_.resize(robot + 1, 0);

    encounteredCollisions_[robot]++;
}

void BacktrackingResolution::saveCollision(const uint32_t _coll)
{
    addCollision(_coll);
}

const std::vector<uint32_t> &BacktrackingResolution::getRobotCollisions() const
{
    return encounteredCollisions_;
}

std::vector<std::reference_wrapper<Vertex>> BacktrackingResolution::resolve(Vertex &_current, Vertex &_next, int32_t _collision)
{
    generatedSubgraphs_.emplace_back();
    foundSolutions_.clear();

    //Triggered when a robot blocks a vertex
    addCollision(_collision);

    //find the potential when the collision robots leaves the segment
    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(_collision, _next.getSegment().getSegmentId());

    //There is no solution if the colliding robot never leaves the segment.
    //Therefore, do nothing when leavePotential is smaller zero
    if (leavePotential >= 0)
    {
        trackBack(_current, _next, _collision, leavePotential);
    }

    resolutionAttemp_++;
    return foundSolutions_;
}

void BacktrackingResolution::trackBack(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //Free the next Vertex for new expansion
    _next.potential = -1;
    _next.collision = -1;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //If backtracking is not possible (we are on the start vertex) try to wait there
    //Additionally Backtracking beond wait Segments is not allowed to be able to solve
    //multi robot scenarios (avoiding n robots in a row)
    //Anyway backtracking beond wait Segments makes no sense, we will only find allready
    //found solutions...
    if (_current.predecessor_ == NULL || _current.isWaitSegment)
    {
        int32_t collision = -1;

        if (route_querry_->checkSegment(_current, 0, _freePotential + 2 * timeoverlap_, robotDiameter_, collision))
        {
            _next.potential = -1;
            _next.collision = -1;
            //_next.successor_ = NULL; //Tell expander to expand the vertex normally

            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;
            current_n.successor_ = &_next; //Tell expander to only expand to next

            foundSolutions_.push_back(current_n);
        }
        else
        {
            if (_collision != collision)
                addCollision(collision);
        }
    }
    else //we are somewhere on the path
    {
        int32_t collision = -1;
        bool vertexFree = route_querry_->checkSegment(*_current.predecessor_, _current.predecessor_->potential - timeoverlap_, _freePotential + 2 * timeoverlap_, robotDiameter_, collision);

        if (vertexFree || collision != -1)
        {
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            next_n.potential = -1;
            next_n.collision = -1;

            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(*(_current.predecessor_)));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;

            //Set References
            current_n.successor_ = &next_n; //Tell expander to only expand to "new next" (with new Potential)
            next_n.predecessor_ = &current_n;
            next_n.successor_ = &_next; //Tell expander to only expand to next (we are not allowed to leave the backtracked path)

            if (vertexFree)
            {
                foundSolutions_.push_back(current_n);
            }
            else if (collision != -1)
            {
                if (collision != -1 && collision != _collision)
                    addCollision(collision);

                float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, current_n.getSegment().getSegmentId());
                trackBack(current_n, next_n, collision, leavePotential);

                //Continue Resolving (Avoid Segment, Avoid Crossing, ...)
            }
        }
    }
}
} // namespace multi_robot_router
