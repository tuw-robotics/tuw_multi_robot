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

#include <tuw_global_router/multi_robot_router.h>
#include <chrono>
#include <iostream>
#include <ros/ros.h>

namespace multi_robot_router
{
//TODO Multithreaded

    MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotDiameter) :  RouteGenerator(), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setRobotNr(_nr_robots);
        robotDiameter_ = _robotDiameter;
        route_coordinator_ = &rct_;
    }
    
    MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots) :  RouteGenerator(), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setRobotNr(_nr_robots);
        std::vector<uint32_t> robotDiameter(_nr_robots, 0);
        robotDiameter_ = robotDiameter;
        route_coordinator_ = &rct_;
    }


    void MultiRobotRouter::setCollisionResolver(const SegmentExpander::CollisionResolverType cRes)
    {
        cResType_ = cRes;
    }


    void MultiRobotRouter::setRobotNr(const uint32_t _nr_robots)
    {
        nr_robots_ = _nr_robots;
        priority_scheduler_.reset(_nr_robots);
    }

    void MultiRobotRouter::setRobotDiameter(const std::vector< uint32_t > &_diameter)
    {
        robotDiameter_.clear();
        robotDiameter_ = _diameter;
        min_diameter_ = std::numeric_limits<uint32_t>::max();

        for(const uint32_t d : robotDiameter_)
        {
            min_diameter_ = std::min(min_diameter_, d);
        }
    }

    void MultiRobotRouter::resetAttempt(const std::vector< Segment > &_graph)
    {
        route_coordinator_->reset(_graph, nr_robots_);
        priority_scheduler_.reset(nr_robots_);
        speed_scheduler_.reset(nr_robots_);
        robotCollisions_.clear();
        robotCollisions_.resize(nr_robots_);
        priorityScheduleAttempts_ = 0;
        speedScheduleAttempts_ = 0;
    }

    const uint32_t MultiRobotRouter::getPriorityScheduleAttempts() const
    {
        return priorityScheduleAttempts_;
    }

    const uint32_t MultiRobotRouter::getSpeedScheduleAttempts() const
    {
        return speedScheduleAttempts_;
    }

    bool MultiRobotRouter::getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector< Checkpoint>> &routingTable, const float &_timeLimit)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> tstart = std::chrono::high_resolution_clock::now();

        resetAttempt(_graph);
        route_coordinator_->setStartSegments(_startSegments);
        route_coordinator_->setGoalSegments(_goalSegments);

        maxIterationsSingleRobot_ =  _graph.size();

        srr.setCollisionResolver(cResType_);
        srr.initSearchGraph(_graph, min_diameter_);

        std::vector<std::vector<RouteVertex>> routeCandidates;
        routeCandidates.resize(nr_robots_);

        std::vector<uint32_t> priorityList = priority_scheduler_.getActualSchedule();
        std::vector<float> speedList = speed_scheduler_.getActualSpeeds();
        uint32_t firstSchedule = 0;
        bool found = false;
        uint32_t lastPlannedRobot;
        float duration = 0;

        do
        {
            int32_t firstRobot = -1;
            do
            {
                //Find first schedule to replan if speed rescheduling was active
                //(used for removing from path coordinator)
                if(firstRobot != -1)
                {
                    for(uint32_t i = 0; i < priorityList.size(); i++)
                    {
                        if(priorityList[i] == firstRobot)
                            firstSchedule = i;
                    }
                }
                found = planPaths(priorityList, speedList, _startSegments, _goalSegments, firstSchedule, routeCandidates, lastPlannedRobot);
            
                speedScheduleAttempts_++;
                std::chrono::time_point<std::chrono::high_resolution_clock>  tgoal = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::milliseconds>(tgoal - tstart).count();
                duration /= 1000;
            }
            while(useSpeedRescheduler_ && duration < _timeLimit && !found && speed_scheduler_.rescheduleSpeeds(lastPlannedRobot, srr.getRobotCollisions(), speedList, firstRobot) );

            priorityScheduleAttempts_++;
        }
        while(usePriorityRescheduler_ && duration < _timeLimit && !found && priority_scheduler_.reschedulePriorities(lastPlannedRobot, srr.getRobotCollisions(), priorityList, firstSchedule));
        routingTable = generatePath(routeCandidates, *route_coordinator_);

        return found;
    }

    bool MultiRobotRouter::planPaths(const std::vector<uint32_t> &_priorityList, const std::vector<float> &_speedList, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, const uint32_t _firstSchedule, std::vector<std::vector<RouteVertex>> &_routeCandidates, uint32_t &_robot)
    {
        bool found = false;

        //Remove only schedules (robots) which have to be replanned
        for(uint32_t i = _firstSchedule; i < nr_robots_; i++)
        {
            _robot = _priorityList[i];
            route_coordinator_->removeRobot(_robot);
        }

        //Find a plan for each robot with no plan
        for(uint32_t i = _firstSchedule; i < nr_robots_; i++)
        {
            found = false;
            _robot = _priorityList[i];

            //route_coordinator_->setActive(_robot);

            RouteCoordinatorWrapper rcWrapper(_robot, *route_coordinator_);
            //Worst case scenario: Search whole graph once + n * (move through whole graph to avoid other robot) -> graph.size() * (i+1) iterations
            if(!srr.getRouteCandidate(_startSegments[_robot], _goalSegments[_robot], rcWrapper, robotDiameter_[_robot], _speedList[_robot], _routeCandidates[_robot], maxIterationsSingleRobot_ * (i + 1)))
            {
                //ROS_INFO("Failed Robot");
                robotCollisions_[_robot] = srr.getRobotCollisions();
                robotCollisions_[_robot].resize(nr_robots_, 0);
                break;
            }

            robotCollisions_[_robot] = srr.getRobotCollisions();
            robotCollisions_[_robot].resize(nr_robots_, 0);

            if(!route_coordinator_->addRoute(_routeCandidates[_robot], robotDiameter_[_robot], _robot))
            {
                ROS_INFO("Failed coordinator");
                break;
            }

            found = true;
        }

        return found;
    }

    void MultiRobotRouter::setPriorityRescheduling(const bool _status)
    {
        usePriorityRescheduler_ = _status;
    }

    void MultiRobotRouter::setSpeedRescheduling(const bool _status)
    {
        useSpeedRescheduler_ = _status;
    }

    
}


