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

#include <tuw_global_planner/segment.h>
#include <ros/ros.h>

Segment::Segment(int _id, const std::vector<Eigen::Vector2d> &_points, const std::vector<int> &_successors, const std::vector<int> &_predecessors, int _width) : 
points_(_points),
successors_(_successors),
predecessors_(_predecessors)
{
    segmentId_ = _id;
    width_ = _width;
    Eigen::Vector2d p = (_points.back() - _points.front());
    length_ = sqrt(p[0]*p[0] + p[1]*p[1]);
}

const Eigen::Vector2d& Segment::getEnd() const
{
    return points_.back();
}

const std::vector< Eigen::Vector2d >& Segment::getPoints() const
{
    return points_;
}

const std::vector< int >& Segment::getPredecessors() const
{
    return predecessors_;
}

int Segment::getSegmentId() const
{
    return segmentId_;
}
const Eigen::Vector2d& Segment::getStart() const
{
    return points_.front();
}
 
const std::vector< int >& Segment::getSuccessors() const
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



Vertex::Vertex(Segment& _seg) : predecessors_(), successors_(), segment_(_seg)
{
    potential = -1;   //Endtime (the time a robot is supposed to leave the segment)
    collision = -1; 
    direction = none;
}

const Segment& Vertex::getSegment()
{
    return segment_;
}

const std::vector< std::reference_wrapper< Vertex > >& Vertex::getPlanningPredecessors() const
{
    return predecessors_;
}

const std::vector< std::reference_wrapper< Vertex > >& Vertex::getPlanningSuccessors() const
{
    return successors_;
}

void Vertex::initNeighbours(std::vector< Vertex >& _sortedVertices)
{
    for(const int & vecId : segment_.getPredecessors())
    {
        Vertex &vRef = _sortedVertices[vecId];
        predecessors_.push_back(vRef);
    }
    
    for(const int & vecId : segment_.getSuccessors())
    {
        Vertex &vRef = _sortedVertices[vecId];
        successors_.push_back(vRef);
    }
}

// bool Vertex::pointOnSegment(Eigen::Vector2d _point)
// {
//     for(const Eigen::Vector2d &it : segment_.getPoints())
//     {
//         if(it[0] == _point[0] && it[1] == _point[1])
//             return true;
//     }
// 
//     return false;
// }

