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

#include <tuw_voronoi_graph/segment.h>
#include <limits>

namespace tuw_graph
{
    int Segment::static_id_ = 0;



    void Segment::cleanNeighbors()
    {
        for(uint32_t i = 0; i < predecessor_.size(); i++)
        {
            if(predecessor_[i] < 0)
            {
                predecessor_.erase(predecessor_.begin() + i);
                i--;
            }
        }
        
        for(uint32_t i = 0; i < successor_.size(); i++)
        {
            if(successor_[i] < 0)
            {
                successor_.erase(successor_.begin() + i);
                i--;
            }
        }
    }

    void Segment::AddPredecessor(const int32_t _predecessor)
    {
        predecessor_.push_back(_predecessor);
    }
    void Segment::AddSuccessor(const int32_t _successor)
    {
        successor_.push_back(_successor);
    }
    Segment::Segment(const std::vector<Eigen::Vector2d> &_points, const float _min_space) : predecessor_(), successor_(), optimizedStart_(false), optimizedEnd_(false)
    {
        if(_points.size() > 0)
        {
            start_ = _points.front();
            end_ = _points.back();
            length_ = _points.size();
            min_space_ = _min_space;
            wayPoints_ = _points;
        }

        id_ = static_id_++;
    }
    Segment::Segment(const uint32_t _id, const std::vector<Eigen::Vector2d> &_points, const float _min_space) : predecessor_(), successor_(), optimizedStart_(false), optimizedEnd_(false)
    {
        if(_points.size() > 0)
        {
            start_ = _points.front();
            end_ = _points.back();
            length_ = _points.size();
            min_space_ = _min_space;
            wayPoints_ = _points;
        }

        id_ = _id;
    }
    void Segment::setStart(Eigen::Vector2d _pt)
    {
        if(wayPoints_.size() == 0)
          wayPoints_.emplace_back(_pt);
        wayPoints_[0] = _pt;
        start_ = _pt;
    }
    void Segment::setEnd(Eigen::Vector2d _pt)
    {
        while(wayPoints_.size() <= 1)
        {
          wayPoints_.emplace_back(_pt);
        }
        wayPoints_[wayPoints_.size() - 1] = _pt;
        end_ = _pt;
    }

    uint32_t Segment::GetId() const
    {
        return id_;
    }

    void Segment::SetId(int _id)
    {
        id_ = _id;
    }

    Eigen::Vector2d Segment::getEnd() const
    {
        return end_;
    }

    Eigen::Vector2d Segment::getStart() const
    {
        return start_;
    }

    std::vector< int32_t > Segment::GetPredecessors() const
    {
        return predecessor_;
    }

    std::vector< int32_t > Segment::GetSuccessors() const
    {
        return successor_;
    }

    bool Segment::ContainsPredecessor(const uint32_t _predecessor)
    {
        for(const auto & pred : predecessor_)
        {
            if(pred == (int32_t)_predecessor)
                return true;
        }

        return false;
    }


    bool Segment::ContainsSuccessor(const uint32_t _successor)
    {
        for(uint32_t i = 0; i < successor_.size(); i++)
        {
            if(successor_[i] == (int32_t)_successor)
                return true;
        }

        return false;
    }


    void Segment::ResetId()
    {
        static_id_ = 0;
    }

    std::vector< Eigen::Vector2d > Segment::GetPath() const
    {
        return wayPoints_;
    }

    void Segment::SetPath(const std::vector< Eigen::Vector2d >  &_points)
    {
        if(_points.size() > 0)
        {
            start_ = _points.front();
            end_ = _points.back();
            length_ = _points.size();
            wayPoints_ = _points;
        }
    }

    float Segment::GetMinPathSpace() const
    {
        return min_space_;
    }

    void Segment::SetMinPathSpace(float _space)
    {
        min_space_ = _space;
    }

    int Segment::GetLength() const
    {
        return length_;
    }

    bool &Segment::getOptStart()
    {
        return optimizedStart_;
    }

    bool &Segment::getOptEnd()
    {
        return optimizedEnd_;
    }
}
