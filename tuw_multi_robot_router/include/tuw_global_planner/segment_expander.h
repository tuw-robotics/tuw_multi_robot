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

#ifndef SEGMENT_EXPANDER_H
#define SEGMENT_EXPANDER_H

#define POT_HIGH 1.0e10
#include <tuw_global_planner/heuristic.h>
#include <tuw_global_planner/path_coordinator.h>
#include <tuw_global_planner/collision_resolution.h>
#include <algorithm>
#include <memory>
#include <queue>

class SegmentExpander
{

private:        template <class T, class S, class C>
                void clearpq(std::priority_queue<T, S, C>& q){
                    q=std::priority_queue<T, S, C>();
                }

private:        struct greaterSegmentWrapper{
                    bool operator()(const std::shared_ptr<Vertex> & _a, const std::shared_ptr<Vertex>& _b) const
                    {
                      return _a->planning.Weight > _b->planning.Weight;
                    }
                };
                

public:         SegmentExpander(std::shared_ptr<Heuristic> _h, std::shared_ptr<PotentialCalculator> _pCalc, std::shared_ptr<Path_Coordinator> _p, std::shared_ptr<CollisionResolution> _cr);
public:         bool calculatePotentials(std::shared_ptr<Vertex> _start, std::shared_ptr<Vertex> _end, std::vector< std::shared_ptr<Vertex> > _graph, float _radius);
private:        std::shared_ptr<Vertex> expandVoronoi(std::shared_ptr<Vertex> _start, std::shared_ptr<Vertex> _end, int _cycles);
private:        void addVoronoiExpansoionCandidate(std::shared_ptr<Vertex> _current, std::shared_ptr<Vertex> _next, std::shared_ptr<Vertex> _end);
private:		void resolveStartCollision(std::shared_ptr<Vertex> _start,std::shared_ptr<Vertex> _end);

private:        std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, greaterSegmentWrapper> seg_queue_;
private:        int neutral_cost_ = 1;
private:        int radius_;
private:        bool path_optimization_ = false;    //Shorter paths but not good for voronoi global


private:        std::shared_ptr<Heuristic> hx_;
private:        std::shared_ptr<PotentialCalculator> pCalc_;
private:        std::shared_ptr<Path_Coordinator> path_querry_;
private:        std::shared_ptr<CollisionResolution> collision_resolution_;
private:	std::vector<int> collisions_robots_;
};

#endif 
