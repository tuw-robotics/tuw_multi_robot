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

#ifndef COLLISION_RESOLUTION_H
#define COLLISION_RESOLUTION_H

#include <tuw_global_router/srr_utils.h>
#include <tuw_global_router/route_coordinator.h>
#include <tuw_global_router/potential_calculator.h>
//TODO
//remember Vertex
//remember Collisions

namespace multi_robot_router
{
class CollisionResolution
{
  public:
    /**
            * @brief resets the session (setting the new route querry and potential calculator)
            * @param _route_querry the route coordinator to coordinate paths
            * @param _pCalc the potential calculator for assigning potential to expanded Vertices
            * @param _robot_radius the radius of the current robot
            */
    virtual void resetSession(const RouteCoordinatorWrapper *_route_querry, const PotentialCalculator *_pCalc, const uint32_t _robot_radius) = 0;
    /**
             * @brief resolves a found collision between two robots. 
             * @param _current the last expanded vertex 
             * @param _next the vertex to expand to 
             * @param _collision the index of the colliding robot
             * @returns a vector of references to Vertices where the Potential Expander can continue expanding
             */
    virtual std::vector<std::reference_wrapper<Vertex>> resolve(Vertex &_current, Vertex &_next, int32_t _collision) = 0;
    /**
             * @brief returns amount of robot collisions found in each resolve try after resetSession
             * @returns a std::vector where the index is the robot index and the value the number of collisions with this robot 
             */
    virtual const std::vector<uint32_t> &getRobotCollisions() const = 0;
    /**
             * @brief increases the collision count of one robot
             * @param _coll the robot elected for increasing its collisions
             */
    virtual void saveCollision(const uint32_t _coll) = 0;
};
} // namespace multi_robot_router
#endif // HEURISTIC_H
