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

#ifndef SPEED_SCHEDULER_H
#define SPEED_SCHEDULER_H

#include <memory>
#include <vector>

namespace multi_robot_router
{
    class SpeedScheduler
    {
      public: 
        /**
         * @brief constructor
         * @param _nrRobots the number of robots used for speed rescheduling
         */    
        SpeedScheduler(const uint32_t _nrRobots);
        /**
         * @brief resets the speed rescheduler (all speeds to max)
         * @param _nrRobots the number of robots used for speed rescheduling 
         */
        void reset(const uint32_t  _nrRobots);
        /**
          * @brief reduces a selected robots maximum speed 
          * @param _collidingRobot the robot which has failed to find a path (therefore a robot with higher priority has to reduce its speed)
          * @param _collisions the collisions the _collidiongRobot has encounterd with other ones
          * @param _newSchedule the new speed Schedule
          * @param _firstRobotToReplan the robot with reduced speed
          * @returns if a new speed schedule is found
          */
        bool rescheduleSpeeds(const uint32_t  _collidingRobot, const std::vector< uint32_t > &_collsisions, std::vector< float > &_newSchedule, int32_t &_firstRobotToReplan);
        /**
         * @brief returns the computed speed schedule
         * @returns the speed schedule 
         */
        const std::vector<float> &getActualSpeeds();

        protected:  
          std::vector<std::vector<float>> checkedSchedules_;
          std::vector<float> actualSpeedSchedule_;
    };
}
#endif // HEURISTIC_H
