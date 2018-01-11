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

#include <voronoi_segmentation/segment.h>
#include <limits>

int Segment::static_id_ = 0;

void Segment::AddPredecessor(std::shared_ptr<Segment> _predecessor)
{
  predecessor_.push_back(_predecessor);
}
void Segment::AddSuccessor(std::shared_ptr<Segment> _successor)
{
  successor_.push_back(_successor);
}
Segment::Segment(std::shared_ptr< std::vector< std::pair< int, int > > > _points, float _min_space) : predecessor_(), successor_()
{
  if(_points->size()>0)
  {
	start_ = *(_points->begin());
	end_ = (_points->back());
	length_ = _points->size();
	min_space_ = _min_space;
	wayPoints_ = _points;
  }
  
  id_ = static_id_++;
}

int Segment::GetId()
{
  return id_;
}

std::pair< int, int > Segment::getEnd()
{
  return end_;
}

std::pair< int, int > Segment::getStart()
{
  return start_;
}

std::vector< std::shared_ptr< Segment > > Segment::GetPredecessors()
{
  return predecessor_;
}

std::vector< std::shared_ptr< Segment > > Segment::GetSuccessors()
{
  return successor_;
}

void Segment::ResetId()
{
  static_id_ = 0;
}

std::shared_ptr< std::vector< std::pair< int, int > > > Segment::GetPath()
{
  return wayPoints_;
}

void Segment::SetPath(std::shared_ptr< std::vector< std::pair< int, int > > > _points)
{
  if(_points->size()>0)
  {
	start_ = *(_points->begin());
	end_ = (_points->back());
	length_ = _points->size();
	wayPoints_ = _points;
  }
}

float Segment::GetMinPathSpace()
{
  return min_space_;
}

void Segment::SetMinPathSpace(float _space)
{
  min_space_ = _space;
}

int Segment::GetLength()
{
  return length_;
}


