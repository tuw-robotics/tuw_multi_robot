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

#include <tuw_global_planner/utils.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
// 
class Segment;

class Neighbours
{
public:		Neighbours(int _minSpace);
public:		void addSegment(std::shared_ptr<Segment> _seg);
public:		bool isCrossing() const;
public:		float getSpace() const;
public:		std::vector<std::shared_ptr<Segment>>::const_iterator cbegin() const;
public:		std::vector<std::shared_ptr<Segment>>::const_iterator cend() const;
public:     size_t size() const;
public:		bool contains(std::shared_ptr<Segment> _seg) const;
//public:		std::shared_ptr<Segment> getParent();							//TODO check if necessary

private:	std::vector<std::shared_ptr<Segment>> segments_;
private:	float space_;
};

class Segment
{  
public:     typedef enum direction_t
            {
              none,
              start_to_end,
              end_to_start
            }direction;
public:     typedef struct astar_planning_t
            {
                float Potential;
                float Weight;
                int Collision;
                direction Direction;
                std::shared_ptr<Segment> BacktrackingPredecessor;  
                std::shared_ptr<Segment> BacktrackingSuccessor;  
				bool WaitSeg;
                astar_planning_t(): Potential(-1), Weight(-1), Collision(-1), Direction(none), WaitSeg(false) {}
            }astar_planning;
  
public:     Segment(int _id, float _minSpace, const std::vector< Point > &_points);
public:     Segment();
public:		void addSuccessor(std::shared_ptr<Segment> _succ);
public:		void addPredecessor(std::shared_ptr<Segment> _pred);
public:		const Neighbours& getSuccessors();
public:		const Neighbours& getPredecessors();
public:		float getPathSpace();
public:		int getIndex();
public:		float getLength();
public:		Point getStart();
public:		Point getEnd();
public:     bool pointOnSegment(Point _pt);
public:     bool isEdgeSegment();
public:     bool isPredecessor(std::shared_ptr<Segment> _seg);
public:     bool isSuccessor(std::shared_ptr<Segment> _seg);
public:     astar_planning planning;

private:	int id_;
private:	float length_;
private:	float space_;
private:	std::vector<Point> points_;
private:	Neighbours successors_;
private:	Neighbours predecessors_;
};


#endif