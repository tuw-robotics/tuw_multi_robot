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

#include <tuw_global_router/srr_utils.h>
#include <ros/ros.h>

namespace multi_robot_router
{

Segment::Segment(const uint32_t &_id, const std::vector<Eigen::Vector2d> &_points, const std::vector<uint32_t> &_successors, const std::vector<uint32_t> &_predecessors, const float &_width) : points_(_points),
                                                                                                                                                                                                successors_(_successors),
                                                                                                                                                                                                predecessors_(_predecessors)
{
    segmentId_ = _id;
    width_ = _width;
    Eigen::Vector2d p = (_points.back() - _points.front());
    length_ = sqrt(p[0] * p[0] + p[1] * p[1]);
}

const Eigen::Vector2d &Segment::getEnd() const
{
    return points_.back();
}

const std::vector<Eigen::Vector2d> &Segment::getPoints() const
{
    return points_;
}

const std::vector<uint32_t> &Segment::getPredecessors() const
{
    return predecessors_;
}

uint32_t Segment::getSegmentId() const
{
    return segmentId_;
}
const Eigen::Vector2d &Segment::getStart() const
{
    return points_.front();
}

const std::vector<uint32_t> &Segment::getSuccessors() const
{
    return successors_;
}

float Segment::length() const
{
    return length_;
}

float Segment::width() const
{
    return width_;
}

Vertex::Vertex(const Segment &_seg) : predecessors_(), successors_(), segment_(_seg), potential(-1), collision(-1), crossingPredecessor(false), crossingSuccessor(false), isWaitSegment(false)
{
}

const Segment &Vertex::getSegment() const
{
    return segment_;
}

const std::vector<std::reference_wrapper<Vertex>> &Vertex::getPlanningPredecessors() const
{
    return predecessors_;
}

const std::vector<std::reference_wrapper<Vertex>> &Vertex::getPlanningSuccessors() const
{
    return successors_;
}

void Vertex::initNeighbours(std::vector<std::unique_ptr<Vertex>> &_sortedVertices, const uint32_t _minSegmentWidth)
{
    for (const uint32_t &vecId : segment_.getPredecessors())
    {
        //Use only segments which can be used by any robot
        if (_sortedVertices[vecId]->getSegment().width() >= _minSegmentWidth)
        {
            Vertex &vRef = *(_sortedVertices[vecId].get());
            predecessors_.push_back(vRef);
        }
    }

    if (predecessors_.size() > 1)
    {
        crossingPredecessor = true;
    }

    for (const uint32_t &vecId : segment_.getSuccessors())
    {
        //Use only segments which can be used by any robot
        if (_sortedVertices[vecId]->getSegment().width() >= _minSegmentWidth)
        {
            Vertex &vRef = *(_sortedVertices[vecId]);
            successors_.push_back(vRef);
        }
    }

    if (successors_.size() > 1)
    {
        crossingSuccessor = true;
    }
}

void Vertex::updateVertex(const Vertex &_v)
{
    potential = _v.potential;
    collision = _v.collision;
    weight = _v.weight;
    //direction = _v.direction;
    predecessor_ = _v.predecessor_;
}

RouteVertex::RouteVertex(const Vertex &_vertex) : segment_(_vertex.getSegment()), direction(path_direction::none), potential(_vertex.potential), collision(_vertex.collision), overlapPredecessor(_vertex.crossingPredecessor), overlapSuccessor(_vertex.crossingSuccessor)
{
}
RouteVertex::RouteVertex(const RouteVertex &_vertex) : segment_(_vertex.getSegment()), direction(_vertex.direction), potential(_vertex.potential), collision(_vertex.collision), overlapPredecessor(_vertex.overlapPredecessor), overlapSuccessor(_vertex.overlapSuccessor)
{
}
const Segment &RouteVertex::getSegment() const
{
    return segment_;
}

} // namespace multi_robot_router
