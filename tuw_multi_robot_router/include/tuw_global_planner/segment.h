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

#ifndef SEGMENT_H
#define SEGMENT_H

#include <memory>
#include <vector>
#include <algorithm>
#include <functional>
#include <eigen3/Eigen/Dense>

class Segment
{
public:
  Segment(int _id, const std::vector<Eigen::Vector2d> &_points, const std::vector<int> &_successors, const std::vector<int> &_predecessors, int _width);
  int getSegmentId() const;
  float width() const;
  float length() const;
  
  const std::vector<Eigen::Vector2d> &getPoints() const;
  const std::vector<int> &getPredecessors() const;
  const std::vector<int> &getSuccessors() const;
  
  const Eigen::Vector2d &getStart() const;
  const Eigen::Vector2d &getEnd() const;
private:
  int segmentId_;
  float width_;
  float length_;
  std::vector<Eigen::Vector2d> points_;
  std::vector<int> predecessors_;
  std::vector<int> successors_;
};


class Vertex
{
public:
  typedef enum direction_t
  {
    none,
    start_to_end,
    end_to_start
  }path_direction;
  
  Vertex(Segment &_seg);
  const std::vector<std::reference_wrapper<Vertex>> &getPlanningSuccessors() const;
  const std::vector<std::reference_wrapper<Vertex>> &getPlanningPredecessors() const;
  
  void initNeighbours(std::vector<Vertex> &_sortedVertices);
  
  const Segment &getSegment();
//   bool pointOnSegment(Eigen::Vector2d _point);  
  
  int potential;   //Endtime (the time a robot is supposed to leave the segment)
  int collision; 
  path_direction direction;
private:
  std::vector<std::reference_wrapper<Vertex>> successors_; 
  std::vector<std::reference_wrapper<Vertex>> predecessors_;   
  Segment &segment_;
};

class Checkpoint
{
public:
  typedef struct Precondition_t
  {
      int robotId;
      int stepCondition;
  }Precondition;
  
  Checkpoint(Segment &_seg);
  const Segment &getSegment();
  std::vector<Precondition> preconditions;
private:
  Segment &segment_;
};


#endif