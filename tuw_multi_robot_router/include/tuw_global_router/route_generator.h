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

#ifndef ROUTE_GENERATOR_H
#define ROUTE_GENERATOR_H

#include <tuw_global_router/srr_utils.h>
#include <tuw_global_router/mrr_utils.h>
#include <tuw_global_router/route_coordinator.h>
#include <eigen3/Eigen/Dense>

namespace multi_robot_router
{
class RouteGenerator
{
  public:
    /** 
             * @brief generates a final Routing Table containing Segment List and Preconditions to other robots
             * @param _path the list of route candidates found
             * @param routeQuerry_ the route coordinator, used to find _path 
             */
    std::vector<std::vector<Checkpoint>> generatePath(const std::vector<std::vector<RouteVertex>> &_paths, const RouteCoordinator &routeQuerry_) const;

  private:
    //creates a Checkpoint from a route Vertex
    Checkpoint createElement(const RouteVertex &_element) const;
    //Analyzes a segment for preconditions depending a specific robot
    void addPreconditions(Checkpoint &_segment, const RouteVertex &_segToFind, const uint32_t _pathNr, const std::vector<std::vector<RouteVertex>> &_paths, const RouteCoordinator &routeQuerry_) const;
};
} // namespace multi_robot_router

#endif
