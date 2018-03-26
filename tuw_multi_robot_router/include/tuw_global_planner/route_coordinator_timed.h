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

#ifndef ROUTE_COORDINATOR_TIMELESS_H
#define ROUTE_COORDINATOR_TIMELESS_H

#include <tuw_global_planner/route_coordinator.h>
#include <tuw_global_planner/srr_utils.h>



namespace multi_robot_router
{

    class RouteCoordinatorTimed :  public RouteCoordinator
    {
            class Timeline
            {
                    typedef struct seg_occupation_t
                    {
                        uint32_t robot;
                        float spaceOccupied;
                        uint32_t startTime;
                        int32_t endTime;                          //-1 means forever
                        bool mainSeg;
                        seg_occupation_t(const uint32_t _robot, const float &_spaceOcupied, const uint32_t _startTime, const int32_t _endTime): robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(true)
                        {
                        }
                        seg_occupation_t(const uint32_t _robot, const float &_spaceOcupied, const uint32_t _startTime, const int32_t _endTime, const bool &_mainSeg): robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(_mainSeg)
                        {
                        }
                    } seg_occupation;

                public:
                    Timeline();

                    void reset(const std::vector< Segment > &_graph, const uint32_t _nrRobots);
                    bool addSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, bool _mainSeg);
                    bool checkSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, int32_t &_lastCollisionRobot) const;
                    bool addCrossingSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, const bool &_mainSeg);
                    bool checkCrossingSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, int32_t &_lastCollisionRobot) const;
                    int32_t findSegId(const int32_t _robot, const uint32_t _timestep) const;
                    uint32_t getSize() const;
                    int32_t getTimeUntilRobotOnSegment(const int32_t _robotNr, const uint32_t _segId) const;
                    std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
                    void removeRobot(const uint32_t _robot);

                private:
                    std::vector<std::vector<seg_occupation>> timeline_;
                    std::vector<std::vector<uint32_t>> robotSegments_;
                    std::vector<float> segmentSpace_;
                    uint32_t maxTime_ = 0;
                    uint32_t nrRobots_ = 0;

                    uint32_t tmpRobot_ = 0;
            };


        public:
            RouteCoordinatorTimed();
            void reset(const std::vector<Segment>  &_graph, const uint32_t _nrRobots);
            bool addRoute(const std::vector< RouteVertex > &_path, const uint32_t _diameterPixel);
            bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, bool _ignoreGoal = false) const;
            void setActive(const uint32_t _robotNr);
            bool setGoalSegments(const std::vector<uint32_t> &_goalSegments);
            bool setStartSegments(const std::vector<uint32_t> &_startSegments);
            bool isGoal(const Vertex &_seg) const;
            const uint32_t getStart() const;
            const uint32_t  getEnd() const;
            int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const;
            int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const;        //-1 means forever
            std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
            void removeRobot(const uint32_t _robot);
        private:
            bool checkSegmentSingle(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, const bool &_ignoreGoal) const;


            std::vector< uint32_t > robotSize_;
            uint32_t activeRobot_ = 0;
            Timeline timeline_;
            std::vector<uint32_t> goalSegments_;
            std::vector<uint32_t> startSegments_;

    };
}
#endif // PATH_COORDINATOR_TIMELESS_H
