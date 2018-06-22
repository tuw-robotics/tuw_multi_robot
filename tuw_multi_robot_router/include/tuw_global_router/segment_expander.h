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
#include <tuw_global_router/heuristic.h>
#include <tuw_global_router/route_coordinator.h>
#include <tuw_global_router/collision_resolution.h>
#include <tuw_global_router/avoidance_resolution.h>
#include <tuw_global_router/backtracking_resolution.h>
#include <tuw_global_router/empty_resolution.h>
#include <algorithm>
#include <memory>
#include <queue>

namespace multi_robot_router
{
class SegmentExpander
{
  public:
    enum class CollisionResolverType
    {
        none,
        backtracking,
        avoidance
    };
    /**
             * @brief constructor
             * @param _h the heuristic used (A-Star, Dijkstra, ...)
             * @param _pCalc the used Potential Calculator for the expander
             * @param _cRes the used collision resolution strategy 
             */
    SegmentExpander(const CollisionResolverType _cRes);
    /**
             * @brief assigns all Vertices in the Search graph with a potential according to the distance to the start
             * @param _p the route coordinator to coordinate the route with other robots 
             * @param _start the start Vertex 
             * @param _end the end Vertex 
             * @returns if the goal was found
            */
    bool calculatePotentials(const RouteCoordinatorWrapper *_p, Vertex &_start, Vertex &_end, const uint32_t _maxIterations, const uint32_t _radius);
    /**
             * @brief returns the found robotCollisions while planning 
             */
    const std::vector<uint32_t> &getRobotCollisions() const;
    /**
             * @brief resets the session 
             */
    void reset();
    /**
             * @brief Sets the multiplier to reduce a robots speed (pCalc...)
             */
    void setSpeed(const float &_speed);
    /**
             * @brief sets the desired collision resolver  
             */
    void setCollisionResolver(const CollisionResolverType cRes);
    /**
             * @brief gets the currently used collision resolver 
             */
    CollisionResolverType getCollisionResolver() const;

  private:
    template <class T, class S, class C>
    void clearpq(std::priority_queue<T, S, C> &q)
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

    //assigns the potential to all necessary vertices
    Vertex *expandVoronoi(Vertex &_start, Vertex &_end, const uint32_t _cycles);
    //assigns the next Vertex with the given potential
    void addExpansoionCandidate(Vertex &_current, Vertex &_next, Vertex &_end);
    //adds teh first expenstion candidate
    void addStartExpansionCandidate(Vertex &_start, Vertex &_current, Vertex &_next, Vertex &_end);
    //Resolves start issues when start and goal point is on the same segment
    void resolveStartCollision(Vertex &_start, Vertex &_end);
    //checks if a list contains a specific vertex (seg id)
    bool containsVertex(const Vertex &_v, const std::vector<std::reference_wrapper<Vertex>> &_list) const;

    std::priority_queue<Vertex *, std::vector<Vertex *>, greaterSegmentWrapper> seg_queue_;
    uint32_t neutral_cost_ = 1;
    uint32_t diameter_;

    Heuristic hx_;
    PotentialCalculator pCalc_;
    CollisionResolution *collision_resolution_;
    AvoidanceResolution avr_;
    BacktrackingResolution btr_;
    EmptyResolution er_;

    const RouteCoordinatorWrapper *route_querry_;
    std::vector<uint32_t> collisions_robots_;
    std::vector<std::unique_ptr<Vertex>> startSegments_;
    CollisionResolverType crType_;
};
} // namespace multi_robot_router
#endif
