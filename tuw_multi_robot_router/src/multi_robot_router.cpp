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

#include <tuw_global_planner/multi_robot_router.h>
#include <tuw_global_planner/single_robot_router.h>
#include <iostream>
#include <ros/ros.h>

//TODO Timelimit

MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotRadius) : RouteGenerator(), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
{
    route_coordinator_ = std::make_unique<RouteCoordinatorTimed>();
    setRobotNr(_nr_robots);
    robotDiameter_ = _robotRadius;
}

void MultiRobotRouter::setRobotNr(const uint32_t _nr_robots)
{
    nr_robots_ = _nr_robots;
    priority_scheduler_.reset(_nr_robots);
}

void MultiRobotRouter::setRobotRadius(const std::vector< uint32_t > &_diameter)
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

bool MultiRobotRouter::getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &startSegments, const std::vector<uint32_t> &goalSegments, std::vector<std::vector< Checkpoint>> &routingTable)
{
    resetAttempt(_graph);
    route_coordinator_->setStartSegments(startSegments);
    route_coordinator_->setGoalSegments(goalSegments);


    SingleRobotRouter srr;
    srr.initSearchGraph(_graph, min_diameter_);

    std::vector<std::vector<RouteVertex>> routeCandidates;
    routeCandidates.resize(nr_robots_);

    std::vector<uint32_t> priorityList = priority_scheduler_.getActualSchedule();
    std::vector<float> speedList = speed_scheduler_.getActualSpeeds();
    uint32_t firstSchedule = 0;
    bool found = false;
    uint32_t robot;
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
          
          
            //Remove only schedules (robots) which have to be replanned
            for(uint32_t i = firstSchedule; i < nr_robots_; i++)
            {
                robot = priorityList[i];
                route_coordinator_->removeRobot(robot);
            }
            
            //Find a plan for each robot with no plan
            for(uint32_t i = firstSchedule; i < nr_robots_; i++)
            {
                found = false;
                robot = priorityList[i];
                
                route_coordinator_->setActive(robot);
                //Worst case scenario: Search whole graph once + n * (move through whole graph to avoid other robot) -> graph.size() * (i+1) iterations
                if(!srr.getRouteCandidate(startSegments[robot], goalSegments[robot], *route_coordinator_.get(), robotDiameter_[robot], speedList[robot], routeCandidates[robot], _graph.size() * (i + 1)))
                {
                    //ROS_INFO("Failed Robot");
                    robotCollisions_[robot] = srr.getRobotCollisions();
                    robotCollisions_[robot].resize(nr_robots_,0);
                    break;
                }
                robotCollisions_[robot] = srr.getRobotCollisions();
                robotCollisions_[robot].resize(nr_robots_,0);
                
                if(!route_coordinator_->addRoute(routeCandidates[robot], robotDiameter_[robot]))
                {
                    ROS_INFO("Failed coordinator");
                    break;
                }

                found = true;
                speedScheduleAttempts_++;
            }
        }while(!found && speed_scheduler_.rescheduleSpeeds(robot,srr.getRobotCollisions(), speedList, firstRobot));
        
        priorityScheduleAttempts_++;
    }while(!found && priority_scheduler_.reschedulePriorities(robot, srr.getRobotCollisions(), priorityList, firstSchedule));
    
    routingTable = generatePath(routeCandidates, *(route_coordinator_.get()));

    return found;
}

