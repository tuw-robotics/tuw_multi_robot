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
#include <tuw_global_planner/route_coordinator.h>
#include <tuw_global_planner/collision_resolution.h>
#include <tuw_global_planner/backtracking_avoid_resolution.h>
#include <algorithm>
#include <memory>
#include <queue>

class SegmentExpander
{
    public:
        SegmentExpander(const Heuristic &_h, const PotentialCalculator &_pCalc);//, const CollisionResolution &_cr);
        bool calculatePotentials(const RouteCoordinator *_p, Vertex &_start, Vertex &_end, const uint32_t _maxIterations, const uint32_t _radius);
        const std::vector<uint32_t> &getRobotCollisions() const;
        void reset();
        void setSpeed(const float &_speed);
    private:
        template <class T, class S, class C>
        void clearpq(std::priority_queue<T, S, C>& q)
        {
            q = std::priority_queue<T, S, C>();
        }

        struct greaterSegmentWrapper
        {
            bool operator()(const Vertex *_a, const Vertex *_b) const
            {
                return _a->weight > _b->weight;
            }
        };


        Vertex *expandVoronoi(Vertex &_start, Vertex &_end, const uint32_t _cycles);
        void addVoronoiExpansoionCandidate(Vertex &_current, Vertex &_next, Vertex &_end);
        void resolveStartCollision(Vertex &_start, Vertex &_end);

        bool containsVertex(const Vertex &_v, const std::vector< std::reference_wrapper< Vertex > >& _list) const;
        
        std::priority_queue<Vertex*, std::vector<Vertex*>, greaterSegmentWrapper> seg_queue_;
        uint32_t neutral_cost_ = 1;
        uint32_t diameter_;


        std::unique_ptr<Heuristic> hx_;
        std::unique_ptr<PotentialCalculator> pCalc_;
        const RouteCoordinator *route_querry_;
        BacktrackingAvoidResolution collision_resolution_;

        std::vector<uint32_t> collisions_robots_;
};

#endif
