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

#include <tuw_global_router/priority_scheduler.h>

namespace multi_robot_router
{
PriorityScheduler::PriorityScheduler(const uint32_t _nrRobots)
{
    reset(_nrRobots);
}

void PriorityScheduler::reset(const uint32_t _nrRobots)
{
    checkedSchedules_.clear();
    actualPrioritySchedule_.clear();

    for (uint32_t i = 0; i < _nrRobots; i++)
    {
        actualPrioritySchedule_.emplace_back(i);
    }

    checkedSchedules_.emplace_back(actualPrioritySchedule_);
}

const std::vector<uint32_t> &PriorityScheduler::getActualSchedule() const
{
    return actualPrioritySchedule_;
}

bool PriorityScheduler::reschedulePriorities(const uint32_t _collidingRobot, std::vector<uint32_t> _collisions, std::vector<uint32_t> &_newSchedule, uint32_t &_firstRobotToReplan)
{
    _collisions.resize(_newSchedule.size(), 0);

    auto it = std::find(actualPrioritySchedule_.begin(), actualPrioritySchedule_.end(), _collidingRobot);

    if (it == actualPrioritySchedule_.end())
        return false;

    uint32_t priorityCollidingRobot = std::distance(actualPrioritySchedule_.begin(), it);

    bool found;
    uint32_t count = 0;
    std::vector<uint32_t> newSchedule = actualPrioritySchedule_;

    do
    {
        found = true;
        uint32_t maxCollisions = 0;
        bool robotFound = false;
        _firstRobotToReplan = 0;

        for (uint32_t i = 0; i < priorityCollidingRobot; i++)
        {
            uint32_t robot = newSchedule[i];

            if (_collisions[robot] > maxCollisions)
            {
                maxCollisions = _collisions[robot];
                _firstRobotToReplan = i;
                robotFound = true;
            }
        }

        if (!robotFound)
            return false;

        std::swap(newSchedule[_firstRobotToReplan], newSchedule[priorityCollidingRobot]);

        for (const std::vector<uint32_t> &schedule : checkedSchedules_)
        {
            bool neq = false;

            for (uint32_t i = 0; i < newSchedule.size(); i++)
            {
                if (newSchedule[i] != schedule[i])
                {
                    neq = true;
                    break;
                }
            }

            if (neq == false)
            {
                found = false;
                //Swap back
                std::swap(newSchedule[_firstRobotToReplan], newSchedule[priorityCollidingRobot]);
                _collisions[newSchedule[_firstRobotToReplan]] = 0; //Remove robot from potential exchange list
                break;
            }
        }

        count++;
    } while (count < priorityCollidingRobot && !found);

    if (found)
    {
        actualPrioritySchedule_ = newSchedule;
        _newSchedule = newSchedule;
        checkedSchedules_.emplace_back(actualPrioritySchedule_);
        return true;
    }

    return false;
}
} // namespace multi_robot_router
