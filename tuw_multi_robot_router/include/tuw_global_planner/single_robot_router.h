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

#ifndef SRR_H
#define SRR_H

#include <tuw_global_planner/srr_utils.h>
#include <tuw_global_planner/route_coordinator.h>
#include <tuw_global_planner/segment_expander.h>
#include <tuw_global_planner/traceback.h>
#include <iostream>

typedef struct Robot
{
    int id;
    float diameter;
    float speedMultiplier;
    Robot(int _id, float _d, float _s) : id(_id), diameter(_d), speedMultiplier(_s){}
    Robot(int _id, float _d) : Robot(_id, _d, 1){}
}Robot;


class SingleRobotRouter
{
    public:
        SingleRobotRouter();
        bool getRouteCandidate(const uint32_t _start, const uint32_t _goal, const RouteCoordinator &path_coordinator, const uint32_t _radius, std::vector<RouteVertex> &path);
        const std::vector<uint32_t> &getRobotCollisions() const;
        void initSearchGraph(const std::vector<Segment> &_graph);
    private:
        void resetAttempt();
        std::unique_ptr<SegmentExpander> segment_expander_;
        std::unique_ptr<Traceback> traceback_;
        uint32_t radius_;
        std::vector<Vertex> searchGraph_;
        
};

#endif
