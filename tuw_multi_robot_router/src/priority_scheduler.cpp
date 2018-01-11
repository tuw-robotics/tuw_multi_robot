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

#include <tuw_global_planner/priority_scheduler.h>

std::vector< int >& PriorityScheduler::getActualSchedule()
{
    return actualPrioritySchedule_;
}

PriorityScheduler::PriorityScheduler(int _nrRobots)
{
    reset(_nrRobots);
}

bool PriorityScheduler::reschedulePriorities(int _collidingRobot, const std::vector< int > _collsisions, std::vector< int >& _newSchedule)
{
    auto it = std::find(actualPrioritySchedule_.begin(), actualPrioritySchedule_.end(), _collidingRobot);

    int priorityCollidingRobot = -1;

    if(it != actualPrioritySchedule_.end())
    {
        priorityCollidingRobot = std::distance(actualPrioritySchedule_.begin(), it);
    }

    if(priorityCollidingRobot <= 0)
        return false;

    while(priorityCollidingRobot > 0)
    {
        int priorityOtherElement = priorityCollidingRobot;
        priorityCollidingRobot--;

        std::swap(actualPrioritySchedule_[priorityOtherElement], actualPrioritySchedule_[priorityCollidingRobot]);
        int otherElem = actualPrioritySchedule_[priorityOtherElement];


        if(_collsisions[otherElem] > 0)
        {
            //check if allready used
            bool notEqual = true;

            for(auto & schedule : checkedSchedules_)
            {
                notEqual = false;

                for(int i = 0; i < schedule.size(); i++)
                {
                    if(schedule[i] != actualPrioritySchedule_[i])
                    {
                        notEqual = true;
                    }
                }

                if(!notEqual)
                    break;
            }

            if(notEqual)
            {
                _newSchedule = actualPrioritySchedule_;
                checkedSchedules_.emplace_back(actualPrioritySchedule_);
                return true;
            }
        }
    }

    return false;
}

void PriorityScheduler::reset(int _nrRobots)
{
    checkedSchedules_.clear();

    actualPrioritySchedule_.clear();

    for(int i = 0; i < _nrRobots; i++)
    {
        actualPrioritySchedule_.push_back(i);
    }

    checkedSchedules_.emplace_back(actualPrioritySchedule_);
}

