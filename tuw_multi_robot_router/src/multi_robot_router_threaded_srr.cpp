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

#include <tuw_global_router/multi_robot_router_threaded_srr.h>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <thread>

namespace multi_robot_router
{
    MultiRobotRouterThreadedSrr::MultiRobotRouterThreadedSrr(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotDiameter, const uint32_t _threads) : MultiRobotRouter(_nr_robots, _robotDiameter), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setThreads(_threads);
        setRobotNr(_nr_robots);
        robotDiameter_ = _robotDiameter;
        route_coordinator_ = &rct_;

        for(uint32_t i = 0; i < threads_; i++)
        {
            SingleRobotRouter sr;
            srr.push_back(sr);
        }
        
    }
    
    MultiRobotRouterThreadedSrr::MultiRobotRouterThreadedSrr(const uint32_t _nr_robots, const uint32_t _threads) : MultiRobotRouter(_nr_robots), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setThreads(_threads);
        setRobotNr(_nr_robots);
        std::vector<uint32_t> robotRadius(_nr_robots, 0);
        robotDiameter_ = robotRadius;
        route_coordinator_ = &rct_;

        for(uint32_t i = 0; i < threads_; i++)
        {
            SingleRobotRouter sr;
            srr.push_back(sr);
        }
    }


    void MultiRobotRouterThreadedSrr::setThreads(const uint32_t _threads)
    {
        threads_ = _threads;
    }
    
    
    void MultiRobotRouterThreadedSrr::setCollisionResolver(const SegmentExpander::CollisionResolverType cRes)
    {
        cResType_ = cRes;
    }


    void MultiRobotRouterThreadedSrr::setRobotNr(const uint32_t _nr_robots)
    {
        nr_robots_ = _nr_robots;
        priority_scheduler_.reset(_nr_robots);
    }

    void MultiRobotRouterThreadedSrr::setRobotDiameter(const std::vector< uint32_t > &_diameter)
    {
        robotDiameter_.clear();
        robotDiameter_ = _diameter;
        min_diameter_ = std::numeric_limits<uint32_t>::max();

        for(const uint32_t d : robotDiameter_)
        {
            min_diameter_ = std::min(min_diameter_, d);
        }
    }

    void MultiRobotRouterThreadedSrr::resetAttempt(const std::vector< Segment > &_graph)
    {
        route_coordinator_->reset(_graph, nr_robots_);
        priority_scheduler_.reset(nr_robots_);
        speed_scheduler_.reset(nr_robots_);
        robotCollisions_.clear();
        robotCollisions_.resize(nr_robots_);
        priorityScheduleAttempts_ = 0;
        speedScheduleAttempts_ = 0;
    }

    const uint32_t MultiRobotRouterThreadedSrr::getPriorityScheduleAttempts() const
    {
        return priorityScheduleAttempts_;
    }

    const uint32_t MultiRobotRouterThreadedSrr::getSpeedScheduleAttempts() const
    {
        return speedScheduleAttempts_;
    }

    bool MultiRobotRouterThreadedSrr::getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector< Checkpoint>> &routingTable, const float &_timeLimit)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> tstart = std::chrono::high_resolution_clock::now();

        resetAttempt(_graph);
        route_coordinator_->setStartSegments(_startSegments);
        route_coordinator_->setGoalSegments(_goalSegments);

        maxIterationsSingleRobot_ =  _graph.size();

        for(SingleRobotRouter & sr : srr)
        {
            sr.setCollisionResolver(cResType_);
            sr.initSearchGraph(_graph, min_diameter_);
        }

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
            while(useSpeedRescheduler_ && duration < _timeLimit && !found && speed_scheduler_.rescheduleSpeeds(lastPlannedRobot, robotCollisions_[lastPlannedRobot], speedList, firstRobot));

            priorityScheduleAttempts_++;
        }
        while(usePriorityRescheduler_ && duration < _timeLimit && !found && priority_scheduler_.reschedulePriorities(lastPlannedRobot, robotCollisions_[lastPlannedRobot], priorityList, firstSchedule));

        routingTable = generatePath(routeCandidates, *route_coordinator_);

        return found;
    }

    //TODO Optimize
    bool MultiRobotRouterThreadedSrr::planPaths(const std::vector<uint32_t> &_priorityList, const std::vector<float> &_speedList, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, const uint32_t _firstSchedule, std::vector<std::vector<RouteVertex>> &_routeCandidates, uint32_t &_lastRobot)
    {
        //Remove only schedules (robots) which have to be replanned
        for(uint32_t i = _firstSchedule; i < nr_robots_; i++)
        {
            uint32_t robot = _priorityList[i];
            route_coordinator_->removeRobot(robot);
        }


        uint32_t plannedRobots = _firstSchedule;

        while(plannedRobots < nr_robots_)
        {
            uint32_t maxCount = std::min(plannedRobots + threads_, nr_robots_);
            uint32_t startCount = plannedRobots;
            int32_t currentFailedI = -1;



            std::vector<std::thread> s;
            s.resize(maxCount - startCount);
            
            std::vector<RouteCoordinatorWrapper> rcWrappers;
            for(int i =  startCount; i < maxCount; i++)
            {
                uint32_t robot = _priorityList[i];
                RouteCoordinatorWrapper rcW(robot, *route_coordinator_);
                rcWrappers.push_back(rcW);
            }

            for(int i = startCount; i < maxCount; i++)
            {
                uint32_t robot = _priorityList[i];
                uint32_t start = _startSegments[robot];
                uint32_t goal = _goalSegments[robot];
                uint32_t diameter = robotDiameter_[robot];
                float speed = _speedList[robot];
                std::vector<RouteVertex> &routeCandidate = _routeCandidates[robot];
                uint32_t iter = maxIterationsSingleRobot_ * (i + 1);
                uint32_t index = i - startCount;
                SingleRobotRouter &sr = srr[index];
                RouteCoordinatorWrapper &wr = rcWrappers[index];
                s[index] = std::thread(                                                                                   //TODO not working multi threaded
                [wr, robot, start, goal, diameter, speed, &routeCandidate, iter, &sr]()
                {
                    sr.getRouteCandidate(start, goal, wr, diameter, speed, routeCandidate, iter);

                });

               // s[index].join();

                //Worst case scenario: Search whole graph once + n * (move through whole graph to avoid other robot) -> graph.size() * (i+1) iterations
                //srr[i - startCount].getRouteCandidate(_startSegments[robot], _goalSegments[robot], rcWrapper, robotDiameter_[robot], _speedList[robot], _routeCandidates[robot], maxIterationsSingleRobot_ * (i + 1));

            }


            for(std::thread & t : s)
            {
                if(t.joinable())
                    t.join();
            }

            for(uint32_t i = startCount; i < maxCount; i++)
            {
                uint32_t robot = _priorityList[i];
                uint32_t planner = i - startCount;
                _lastRobot = robot;

                robotCollisions_[robot] = srr[planner].getRobotCollisions();
                robotCollisions_[robot].resize(nr_robots_, 0);

                if(!srr[planner].getLastResult())
                {
//                     //std::cout << "srr " << i << planner << std::endl;
                    return false;
                }

                if(!route_coordinator_->addRoute(_routeCandidates[robot], robotDiameter_[robot], robot))
                {
                    if(i == startCount)
                    {
                        ROS_INFO("Failed coordinator");
                        return false;
                    }

                    //Replan from plannedRobots
                    break;
                }
                else
                {
                    plannedRobots++;
                }

            }

        }

        return true;
    }

    void MultiRobotRouterThreadedSrr::setPriorityRescheduling(const bool _status)
    {
        usePriorityRescheduler_ = _status;
    }

    void MultiRobotRouterThreadedSrr::setSpeedRescheduling(const bool _status)
    {
        useSpeedRescheduler_ = _status;
    }


}


