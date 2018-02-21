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

Segment::Segment(int _id, float _minSpace, const std::vector< Point >& _points) : successors_(_minSpace), predecessors_(_minSpace), points_(_points)
{
    id_ = _id;
    space_ = _minSpace;
    length_ = _points.size();
}

Segment::Segment() : successors_(-1), predecessors_(-1), points_(0)
{
    id_ = -1;
    space_ = -1;
    length_ = 0;
}

Segment::~Segment(void)
{
    clear();
}

void Segment::clear()
{
    predecessors_.clear();
    successors_.clear();
    planning.BacktrackingPredecessor.reset();
    planning.BacktrackingSuccessor.reset();
    Segment::astar_planning planEmpty;
    planning = planEmpty;
}

void Segment::addPredecessor(std::shared_ptr< Segment > _pred)
{
    predecessors_.addSegment(_pred);
}

void Segment::addSuccessor(std::shared_ptr< Segment > _succ)
{
    successors_.addSegment(_succ);
}

int Segment::getIndex()
{
    return id_;
}

float Segment::getLength()
{
    return length_;
}

float Segment::getPathSpace()
{
    return space_;
}

Point Segment::getEnd()
{
    return points_.back();
}

Point Segment::getStart()
{
    return points_.front();
}

bool Segment::pointOnSegment(Point _pt)
{
    for(const auto & it : points_)
    {
        if(it[0] == _pt[0] && it[1] == _pt[1])
            return true;
    }

    return false;
}

bool Segment::isEdgeSegment()
{
    if(predecessors_.size() == 0 || successors_.size() == 0)
        return true;

    return false;
}



const Neighbours& Segment::getPredecessors()
{
    return predecessors_;
}

const Neighbours& Segment::getSuccessors()
{
    return successors_;
}

bool Segment::isPredecessor(std::shared_ptr<Segment> _seg)
{
    for(auto it = predecessors_.cbegin(); it != predecessors_.cend(); it++)
    {
        if(_seg->getIndex() == (*it)->getIndex())
            return true;
    }

    return false;
}

bool Segment::isSuccessor(std::shared_ptr<Segment> _seg)
{
    for(auto it = successors_.cbegin(); it != successors_.cend(); it++)
    {
        if(_seg->getIndex() == (*it)->getIndex())
            return true;
    }

    return false;
}


Neighbours::~Neighbours(void)
{
    segments_.clear();
}

void Neighbours::clear()
{
    segments_.clear();
}

Neighbours::Neighbours(int _minSpace)
{
    space_ = _minSpace;
}

void Neighbours::addSegment(std::shared_ptr< Segment > _seg)
{
    segments_.push_back(_seg);

    space_ = std::max<float> (_seg->getPathSpace(), space_);
}

std::vector< std::shared_ptr< Segment > >::const_iterator Neighbours::cbegin() const
{
    return segments_.cbegin();
}

std::vector< std::shared_ptr< Segment > >::const_iterator Neighbours::cend() const
{
    return segments_.cend();
}

float Neighbours::getSpace() const
{
    return space_;
}

bool Neighbours::isCrossing() const
{
    return (segments_.size() > 1);
}

size_t Neighbours::size() const
{
    return segments_.size();
}

bool Neighbours::contains(std::shared_ptr<Segment> _seg) const
{
    for(const auto & seg : segments_)
    {
        if(seg->getIndex() == _seg->getIndex())
            return true;
    }

    return false;
}
