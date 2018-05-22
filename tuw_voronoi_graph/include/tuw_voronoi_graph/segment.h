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

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace tuw_graph
{
    class Segment
    {
        public:
            Segment(const std::vector<Eigen::Vector2d> &_points, const float _min_space);
            Segment(const uint32_t _id, const std::vector<Eigen::Vector2d> &_points, const float _min_space);

            void AddPredecessor(const int32_t _predecessor);   
            void AddSuccessor(const int32_t _successor);
            std::vector<Eigen::Vector2d> GetPath() const;
            void SetPath(const std::vector<Eigen::Vector2d> &_path);
            float GetMinPathSpace() const;
            void SetMinPathSpace(float _space);
            int GetLength() const;
            bool ContainsPredecessor(const uint32_t _predecessor);
            bool ContainsSuccessor(const uint32_t _successor);
            static void ResetId();            
            
            Eigen::Vector2d getStart() const;
            Eigen::Vector2d getEnd() const;

            void setStart(Eigen::Vector2d _pt);
            void setEnd(Eigen::Vector2d _pt);

            uint32_t GetId() const;
            void SetId(int _id);
            std::vector<int32_t> GetPredecessors() const;
            std::vector<int32_t> GetSuccessors() const;

            void cleanNeighbors();
            
            bool &getOptStart();
            bool &getOptEnd();

        private:
            Eigen::Vector2d start_, end_;
            float min_space_;
            float length_;
            std::vector<Eigen::Vector2d> wayPoints_;

            std::vector<int32_t> predecessor_;
            std::vector<int32_t> successor_;

            static int static_id_;
            int id_;

            bool optimizedStart_;
            bool optimizedEnd_;

    };
}
#endif // PLANNER_NODE_H
