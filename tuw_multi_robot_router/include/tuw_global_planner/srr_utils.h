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


namespace multi_robot_router
{
    class Segment
    {
        public:
            Segment(const uint32_t &_id, const std::vector<Eigen::Vector2d> &_points, const std::vector<uint32_t> &_successors, const std::vector<uint32_t> &_predecessors, const float &_width);
            uint32_t getSegmentId() const;
            float width() const;
            float length() const;

            const std::vector<Eigen::Vector2d> &getPoints() const;
            const std::vector<uint32_t> &getPredecessors() const;
            const std::vector<uint32_t> &getSuccessors() const;

            const Eigen::Vector2d &getStart() const;
            const Eigen::Vector2d &getEnd() const;
        private:
            uint32_t segmentId_;
            float width_;
            float length_;
            std::vector<Eigen::Vector2d> points_;
            std::vector<uint32_t> predecessors_;
            std::vector<uint32_t> successors_;
    };


    class Vertex
    {
        public:
            Vertex(const Segment &_seg);
            const std::vector<std::reference_wrapper<Vertex>> &getPlanningSuccessors() const;
            const std::vector<std::reference_wrapper<Vertex>> &getPlanningPredecessors() const;

            void initNeighbours(std::vector<std::unique_ptr<Vertex>> &_sortedVertices, const uint32_t _minSegmentWidth = 0);

            const Segment &getSegment() const;
            void updateVertex(const Vertex &_v);

            int32_t potential = 0;            //Endtime (the time a robot is supposed to leave the segment)
            int32_t collision = 0;
            int32_t weight = 0;
            bool crossingPredecessor = false;
            bool crossingSuccessor = false;
            bool isWaitSegment = false;
            Vertex *predecessor_ = NULL;
            Vertex *successor_ = NULL;
        private:
            std::vector<std::reference_wrapper<Vertex>> successors_;
            std::vector<std::reference_wrapper<Vertex>> predecessors_;
            const Segment &segment_;
    };


    class RouteVertex
    {
        public:
            enum class path_direction
            {
                none,
                start_to_end,
                end_to_start
            };
            RouteVertex(const Vertex &_vertex);
            RouteVertex(const RouteVertex &_vertex);
            const Segment &getSegment() const;

            int32_t potential = 0;
            int32_t collision = 0;
            bool overlapPredecessor = false;
            bool overlapSuccessor = false;
            path_direction direction;
            const Segment &segment_;
    };
}

#endif
