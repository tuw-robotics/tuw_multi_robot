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

#include <tuw_global_router/route_coordinator.h>
#include <tuw_global_router/srr_utils.h>

namespace multi_robot_router
{

class RouteCoordinatorTimed : public RouteCoordinator
{
    class Timeline
    {
        typedef struct seg_occupation_t
        {
            uint32_t robot;
            float spaceOccupied;
            uint32_t startTime;
            int32_t endTime; //-1 means forever
            bool mainSeg;
            seg_occupation_t(const uint32_t _robot, const float &_spaceOcupied, const uint32_t _startTime, const int32_t _endTime) : robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(true)
            {
            }
            seg_occupation_t(const uint32_t _robot, const float &_spaceOcupied, const uint32_t _startTime, const int32_t _endTime, const bool &_mainSeg) : robot(_robot), spaceOccupied(_spaceOcupied), startTime(_startTime), endTime(_endTime), mainSeg(_mainSeg)
            {
            }
        } seg_occupation;

      public:
        Timeline();

        /**
                     * @brief resets the planning session of multiple robots
                     * @param _graph the graph
                     * @param _nrRobots the nr of robots to plan
                     */
        void reset(const std::vector<Segment> &_graph, const uint32_t _nrRobots);
        /**
                     * @brief adds a segment to the timeline and checks if the occupation is valid
                     * @param _startTime the time when the robot enters the segment
                     * @param _endTime the time when the robot leaves the segment
                     * @param _segId the segment id
                     * @param _robotNr the robot Id
                     * @param _robotSize the robot Size
                     * @param _mainSeg saves if this is a main segment (A segment which is passed by the robot and not overlapping with the path)
                     * @returns true if a segment is added to the timeline
                     */
        bool addSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, bool _mainSeg);
        /**
                    * @brief checks if a robot can enter a segment at a specific time
                    * @param _startTime the time the current robot enters the segment
                    * @param _endTime the time the current robot leaves the segment
                    * @param _segId the segment to check
                    * @param _robotNr the id of the robot to check
                    * @param _robotSize the diameter of the robot
                    * @param _lastCollisionRobot returns the robot wich causes a collision (if no collision -1)
                    * @returns if the vertex is free for the given timespan
                    */
        bool checkSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, int32_t &_lastCollisionRobot) const;
        /**
                     * @brief adds a segment in a crossing (neighbours > 1) to the timeline and checks if the occupation is valid
                     * @param _startTime the time when the robot enters the segment
                     * @param _endTime the time when the robot leaves the segment
                     * @param _segId the segment id
                     * @param _robotNr the robot Id
                     * @param _robotSize the robot Size
                     * @param _mainSeg saves if this is a main segment (A segment which is passed by the robot and not overlapping with the path)
                     * @returns true if a segment is added to the timeline
                     */
        bool addCrossingSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, const bool &_mainSeg);
        /**
                    * @brief checks if a robot can enter a segment at a specific time if located in a crossing (neighbours > 1)
                    * @param _startTime the time the current robot enters the segment
                    * @param _endTime the time the current robot leaves the segment
                    * @param _segId the segment to check
                    * @param _robotNr the id of the robot to check
                    * @param _robotSize the diameter of the robot
                    * @param _lastCollisionRobot returns the robot wich causes a collision (if no collision -1)
                    * @returns if the vertex is free for the given timespan
                    */
        bool checkCrossingSegment(const uint32_t _startTime, const int32_t _endTime, const uint32_t _segId, const uint32_t _robotNr, const uint32_t _robotSize, int32_t &_lastCollisionRobot) const;
        /**
                    * @brief returns the segment id at which the robot is at timepoint _potential
                    * @param _robot the robot id
                    * @param _potential the timepoint on the robots path execution
                    * @returns the segment where the robot is located at time _potential
                    */
        int32_t findSegId(const int32_t _robot, const uint32_t _timestep) const;
        /**
                     * @brief returns the length of the timeline (max Time)
                     * @returns the least time a segmetn is occupied 
                     */
        uint32_t getSize() const;
        /**
                    * @brief returns the time point, when a robot leaves a vertex
                    * @param _robotNr the robotId
                    * @param _segId the segmentId where the time is searched
                    * @returns the time the robot leaves the segment (-1 means it is a goal Vertex and is occupied forever)
                    */
        int32_t getTimeUntilRobotOnSegment(const int32_t _robotNr, const uint32_t _segId) const;
        /**
                    * @brief returns all robots which pass the segment before the given robot and a given time
                    * @param _robot the robotId
                    * @param _segId the Vertex to check
                    * @param _potential the maximum potential
                    * @returns all robots which pass the vertex befor _robot and before _potential
                    */
        std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
        /**
                    * @brief removes a Robot from the Routing Table
                    * @param _robot the robot to remove
                    */
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
    /**
             * @brief constructor
             */
    RouteCoordinatorTimed();
    /**
             * @brief resets the planning session of multiple robots
             * @param _graph the graph
             * @param _nrRobots the nr of robots to plan
             */
    void reset(const std::vector<Segment> &_graph, const uint32_t _nrRobots);
    /**
             * @brief adds a new route to the Routing table and checks if the Routing Table is valid
             * @param _path the path to add
             * @param _diameterPixel the diameter of the robot which uses the route
             * @param _robotId the robotId
             * @returns if the new Routing table is valid (if false the new route is not saved in the routing table)
             */
    bool addRoute(const std::vector<RouteVertex> &_path, const uint32_t _diameterPixel, const uint32_t _robotId);
    /**
             * @brief checks if a robot can enter a segment at a specific time
             * @param _next the segment to check
             * @param _startTime the time the current robot enters the segment
             * @param _endTime the time the current robot leaves the segment
             * @param _diameterPixel the diameter of the robot
             * @param _collisionRobot returns the robot wich causes a collision (if no collision -1)
             * @param _robotId the id of the robot to check
             * @param _ignoreGoal if the segment is a goal it is not handled as goal if this param is true. (Used if a robot is pushed away from its goal and has to move back)
             * @returns if the vertex is free for the given timespan
             */
    bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, const uint32_t _robotId, bool _ignoreGoal = false) const;
    /**
             * @brief updates the goal segments of the planning attempt
             * @param _goalSegments the goal Segments with length nr_robots
             * @returns if the goal segments are valid
             */
    bool setGoalSegments(const std::vector<uint32_t> &_goalSegments);
    /**
             * @brief updates the start segments of the planning attempt
             * @param _startSegments the start Segments with length nr_robots
             * @returns if the start segments are valid
             */
    bool setStartSegments(const std::vector<uint32_t> &_startSegments);
    /**
             * @brief returns if a Vertex is the goal vertex for a robot
             * @param _seg the segment to check
             * @param _robotId the robot tocheck
             * @returns if the vertex is a goal for robot robotId
             */
    bool isGoal(const Vertex &_seg, const uint32_t _robotId) const;
    /**
             * @brief returns the start of a robot
             */
    const uint32_t getStart(const uint32_t _robotId) const;
    /**
             * @brief returns the Goal of a robot
             */
    const uint32_t getEnd(const uint32_t _robotId) const;
    /**
             * @brief returns the segment id at which the robot is at timepoint _potential
             * @param _robot the robot id
             * @param _potential the timepoint on the robots path execution
             * @returns the segment where the robot is located at time _potential
             */
    int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const;
    /**
             * @brief returns the time point, when a robot leaves a vertex
             * @param _robot the robotId
             * @param _segId the segmentId where the time is searched
             * @returns the time the robot leaves the segment (-1 means it is a goal Vertex and is occupied forever)
             */
    int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const; //-1 means forever
    /**
             * @brief returns all robots which pass the segment before the given robot and a given time
             * @param _robot the robotId
             * @param _segId the Vertex to check
             * @param _potential the maximum potential
             * @returns all robots which pass the vertex befor _robot and before _potential
             */
    std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
    /**
             * @brief removes a Robot from the Routing Table
             * @param _robot the robot to remove
             */
    void removeRobot(const uint32_t _robot);

  private:
    // Checks a segment ignoring Crossings
    bool checkSegmentSingle(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, const uint32_t _robotId, const bool &_ignoreGoal) const;

    std::vector<uint32_t> robotSize_;
    Timeline timeline_;
    std::vector<uint32_t> goalSegments_;
    std::vector<uint32_t> startSegments_;
};
} // namespace multi_robot_router
#endif // PATH_COORDINATOR_TIMELESS_H
