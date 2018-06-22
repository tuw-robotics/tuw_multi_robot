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

#ifndef ROUTE_COORDINATOR_H
#define ROUTE_COORDINATOR_H

#include <tuw_global_router/mrr_utils.h>
#include <memory>
#include <vector>
#include <tuw_global_router/srr_utils.h>

namespace multi_robot_router
{
class RouteCoordinator
{
  public:
    /**
              * @brief resets the planning session of multiple robots
              * @param _graph the graph
              * @param _nrRobots the nr of robots to plan
              */
    virtual void reset(const std::vector<Segment> &_graph, const uint32_t _nrRobots) = 0;
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
    virtual bool addRoute(const std::vector<RouteVertex> &_path, const uint32_t _diameterPixel, const uint32_t _robotId) = 0;
    /**
             * @brief updates the goal segments of the planning attempt
             * @param _goalSegments the goal Segments with length nr_robots
             * @returns if the goal segments are valid
             */
    virtual bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, const uint32_t _robotId, bool _ignoreGoal = false) const = 0;
    /**
             * @brief updates the goal segments of the planning attempt
             * @param _goalSegments the goal Segments with length nr_robots
             * @returns if the goal segments are valid
             */
    virtual bool setGoalSegments(const std::vector<uint32_t> &_goalSegments) = 0;
    /**
             * @brief updates the start segments of the planning attempt
             * @param _startSegments the start Segments with length nr_robots
             * @returns if the start segments are valid
             */
    virtual bool setStartSegments(const std::vector<uint32_t> &_startSegments) = 0;
    /**
             * @brief returns if a Vertex is the goal vertex for a robot
             * @param _seg the segment to check
             * @param _robotId the robot tocheck
             * @returns if the vertex is a goal for robot robotId
             */
    virtual bool isGoal(const Vertex &_seg, const uint32_t _robotId) const = 0;
    /**
             * @brief returns the start of a robot
             */
    virtual const uint32_t getStart(const uint32_t _robotId) const = 0;
    /**
             * @brief returns the Goal of a robot
             */
    virtual const uint32_t getEnd(const uint32_t _robotId) const = 0;
    /**
             * @brief returns all robots which pass the segment before the given robot and a given time
             * @param _robot the robotId
             * @param _segId the Vertex to check
             * @param _potential the maximum potential
             * @returns all robots which pass the vertex befor _robot and before _potential
             */
    virtual std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const = 0;
    /**
             * @brief removes a Robot from the Routing Table
             * @param _robot the robot to remove
             */
    virtual void removeRobot(const uint32_t _robot) = 0;
    /**
             * @brief returns the segment id at which the robot is at timepoint _potential
             * @param _robot the robot id
             * @param _potential the timepoint on the robots path execution
             * @returns the segment where the robot is located at time _potential
             */
    virtual int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const = 0;
    /**
             * @brief returns the time point, when a robot leaves a vertex
             * @param _robot the robotId
             * @param _segId the segmentId where the time is searched
             * @returns the time the robot leaves the segment (-1 means it is a goal Vertex and is occupied forever)
             */
    virtual int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const = 0;
};

class RouteCoordinatorWrapper
{
  public:
    /** 
             * @brief constructor to save the routecoordinator and the current robot
             * @details holds a static reference to the Routecoordinator and enables to call the route Coordinators const methods using the saved robotId 
             */
    RouteCoordinatorWrapper(const uint32_t _robot, const RouteCoordinator &_routeCoordinater);
    /**
             * @brief calls checkSegment of the _routeCoordinator using robot_
             */
    bool checkSegment(const Vertex &_next, const uint32_t _startTime, const int32_t _endTime, const uint32_t _diameterPixel, int32_t &_collisionRobot, bool _ignoreGoal = false) const;
    /**
             * @brief calls isGoal of the _routeCoordinator using robot_
             */
    bool isGoal(const Vertex &_seg) const;
    /**
             * @brief calls getStart of the _routeCoordinator using robot_
             */
    const uint32_t getStart() const;
    /**
             * @brief calls getEnd of the _routeCoordinator using robot_
             */
    const uint32_t getEnd() const;
    /**
             * @brief calls getListOfRobotsHigherPrioritizedRobots of the _routeCoordinator using robot_
             */
    std::vector<std::pair<uint32_t, float>> getListOfRobotsHigherPrioritizedRobots(const uint32_t _robot, const uint32_t _segId, const int32_t _potential) const;
    /**
             * @brief calls findSegNr of the _routeCoordinator using robot_
             */
    int32_t findSegNr(const uint32_t _robot, const uint32_t _potential) const;
    /**
             * @brief calls findPotentialUntilRobotOnSegment of the _routeCoordinator using robot_
             */
    int32_t findPotentialUntilRobotOnSegment(const uint32_t _robot, const uint32_t _segId) const;

  private:
    uint32_t robot_;
    const RouteCoordinator *routeCoordinator_;
};
} // namespace multi_robot_router
#endif // PATH_QUERRY_H
