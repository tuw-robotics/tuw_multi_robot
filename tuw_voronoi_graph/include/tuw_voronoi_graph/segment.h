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
            /** 
             * @brief constructor
             * @param _path a list of points which describes the real shape of the path  
             * @param _min_space the space a robot has at minimum on its space
             */
            Segment(const std::vector<Eigen::Vector2d> &_path, const float _min_space);
            /** 
             * @brief constructor
             * @param _id the id of the segment
             * @param _path a list of points which describes the real shape of the path  
             * @param _min_space the space a robot has at minimum on its space
             */
            Segment(const uint32_t _id, const std::vector<Eigen::Vector2d> &_points, const float _min_space);

            /** 
             * @brief adds a predecessor to the object
             * @param _predecessor the id of the predecessor
             */
            void addPredecessor(const uint32_t _predecessor);   
            /** 
             * @brief adds a successor to the object
             * @param _successor the id of the successor
             */
            void addSuccessor(const uint32_t _successor);  
            /** 
             * @brief returns the path
             * @return a list of points which represents the real path
             */
            std::vector<Eigen::Vector2d> getPath() const;  
            /** 
             * @brief sets a new path of the robot
             * @param _path a list of points representing the real path
             */
            void setPath(const std::vector<Eigen::Vector2d> &_path);  
            /** 
             * @brief returns the minimum space in a segment
             * @return the minimum space on this segment
             */
            float getMinPathSpace() const;  
            /** 
             * @brief sets the minimum space of a segment
             * @param _space the minimum space on this segemtn
             */
            void setMinPathSpace(const float _space);  
            /** 
             * @brief returns the length of the path
             * @return the path length
             */
            int getLength() const;  
            /** 
             * @brief checks if the segment has a predecessor with id _predecessor
             * @param _predecessor the id the fctn is lookiing for
             * @return if the segment contains a predecessor
             */
            bool containsPredecessor(const uint32_t _predecessor);  
            /** 
             * @brief checks if the segment has a predecessor with id _successor
             * @param _successor the id the fctn is looking for
             * @return if the segment contains this successor
             */
            bool containsSuccessor(const uint32_t _successor);  
            /** 
             * @brief resets the id counter (which is used to generate uinique ids)
             */
            static void resetId();            
              
            /** 
             * @brief returns const ref to startpoint
             * @return  const ref to startpoint
             */
            const Eigen::Vector2d &getStart() const;  
            /** 
             * @brief returns  const ref to endpoint
             * @return  const ref to endpoint
             */
            const Eigen::Vector2d &getEnd() const;
  
            /** 
             * @brief sets the startpoint
             * @param _pt startpoint
             */
            void setStart(const Eigen::Vector2d &_pt);  
            /** 
             * @brief sets endpoint
             * @param _pt endpoint
             */
            void setEnd(const Eigen::Vector2d &_pt);
  
            /** 
             * @brief returns the id
             * @return  the id
             */
            uint32_t getId() const;  
            /** 
             * @brief set the id
             * @param _id the id
             */
            void setId(const uint32_t _id);  
            /** 
             * @brief returns a const reference to the predecessors
             * @return  a const reference to the predecessor vector
             */
            const std::vector<uint32_t> &getPredecessors() const;  
            /** 
             * @brief returns a const reference to the successor
             * @return a const reference to the successor
             */
            const std::vector<uint32_t> &getSuccessors() const;
  
            /** 
             * @brief removing all predecessors or successors with  id
             * @param _id the id to remove
             */
            void cleanNeighbors(uint32_t _id);  
            /** 
             * @brief decreases the id and all neighbor ids above or equal _id by one (needed to safely remove entities)
             * @param _id the minimum id to decrease the id by one 
             */
            void decreaseNeighborIdAbove(uint32_t _id);  
            /** 
             * @brief returns a reference to opt start used to save if a segment was allready optimized
             * @return a reference to opt start
             */
            bool &getOptStart();  
            /** 
             * @brief returns a reference to opt end used to save if a segment was allready optimized
             * @return a reference to opt end
             */
            bool &getOptEnd();

        private:
            Eigen::Vector2d start_, end_;
            float min_space_;
            float length_;
            std::vector<Eigen::Vector2d> wayPoints_;

            std::vector<uint32_t> predecessor_;
            std::vector<uint32_t> successor_;

            static uint32_t static_id_;
            uint32_t id_;

            bool optimizedStart_;
            bool optimizedEnd_;

    };
}
#endif // PLANNER_NODE_H
