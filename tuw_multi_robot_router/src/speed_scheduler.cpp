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

#include <tuw_global_planner/speed_scheduler.h>
#include <climits>

std::vector< float >& SpeedScheduler::getActualSpeeds()
{
    for(auto s : actualSpeedSchedule_)
    {
        //ROS_INFO("r%f", s);
    }

    return actualSpeedSchedule_;
}

SpeedScheduler::SpeedScheduler(int _nrRobots)
{
    reset(_nrRobots);
}

bool SpeedScheduler::rescheduleSpeeds(int _collidingRobot, const std::vector< int > _collisions, std::vector< float >& _newSchedule)
{
    bool found = false;

    while(!found)
    {
        int robot = -1;
        int collisions = 0;

        for(int i = 0; i < _collisions.size(); i++)
        {
            if(_collisions[i] > collisions && _collisions[i] < maxColls_ && collisionsRobot_[i] < maxReductions_ && i != _collidingRobot)
            {
                robot = i;
                collisions = _collisions[i];
            }
        }

        if(robot == -1)
            return false;

        actualSpeedSchedule_[robot] += 2.0;


        if(actualSpeedSchedule_[robot] > 6.0)
            return false;

        found = true;
        collisionsRobot_[robot] ++;
        checkedSchedules_.emplace_back(actualSpeedSchedule_);
        _newSchedule = actualSpeedSchedule_;

    }


    return found;

}

void SpeedScheduler::reset(int _nrRobots)
{
    checkedSchedules_.clear();

    actualSpeedSchedule_.clear();

    for(int i = 0; i < _nrRobots; i++)
    {
        actualSpeedSchedule_.push_back(1.0);
    }

    checkedSchedules_.emplace_back(actualSpeedSchedule_);
    maxColls_ = INT_MAX;
    collisionsRobot_.clear();
    collisionsRobot_.resize(_nrRobots, 0);
}

