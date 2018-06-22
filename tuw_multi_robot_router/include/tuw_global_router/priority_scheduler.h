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

#ifndef PRIORITY_SCHEDULER_H
#define PRIORITY_SCHEDULER_H

#include <tuw_global_router/srr_utils.h>

namespace multi_robot_router
{
class PriorityScheduler
{

  public:
    /**
            * @brief constructor
            * @param _nrRobots the number of robots
            */
    PriorityScheduler(const uint32_t _nrRobots);
    /**
             * @brief resets the Priority schedule with an initial priority schedule
             * @param _nrRobots the number of robots
             */
    void reset(const uint32_t _nrRobots);
    /**
             * @brief rescedules priorities depending on the ound collisions and allready tried schedules
             * @details the priority rescheduler exchanges only two priorities by taking the collidiongRobot and the robot with the most collisions, which produces no equal priority scheme to prior ones. Additionally the highest robot priority which has changed is returned.
             * @param _collidingRobot the robot which has to be exchanged with a higher priority one
             * @param _collisions the collisions the _collidiongRobot has encounterd with other ones
             * @param _newSchedule the new priority Schedule
             * @param _firstRobotToReplan the exchanged robot with the highest robot
             * @returns if a new priority schedule is found
             */
    bool reschedulePriorities(const uint32_t _collidingRobot, std::vector<uint32_t> _collsisions, std::vector<uint32_t> &_newSchedule, uint32_t &_firstRobotToReplan);
    /**
             * @brief returns the currently produced speed schedule
             * @returns the computed speed schedule 
             */
    const std::vector<uint32_t> &getActualSchedule() const;

  protected:
    std::vector<std::vector<uint32_t>> checkedSchedules_;
    std::vector<uint32_t> actualPrioritySchedule_;
};
} // namespace multi_robot_router
#endif
