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
#include <tuw_global_planner/velocity_calculator.h>
#include <algorithm>
#include <ros/ros.h>

bool VelocityCalculator::CalculateProfile(std::vector< std::vector< PathSegment > > _pathsWithPreconditions, std::vector< std::vector< float > >& _relativeVelocityProfile)
{
    paths_ = _pathsWithPreconditions;
    _relativeVelocityProfile.clear();

    if(pathStep_.size() != paths_.size())
        return false;

    for(int i = 0; i < pathStep_.size(); i++)
    {
        if(!MovePath(paths_[i], i))
            return false;
    }

    for(int i = 0; i < actualPathSteps_.size(); i++)
    {
        _relativeVelocityProfile.emplace_back();
        int lastIndex = 0;
        float pathLength = 0;

        for(int j = 0; j < actualPathSteps_[i].size(); j++)
        {
            _relativeVelocityProfile[i].push_back(1.0);

            float dist_x = _pathsWithPreconditions[i][j].start.x - _pathsWithPreconditions[i][j].end.x;
            float dist_y = _pathsWithPreconditions[i][j].start.y - _pathsWithPreconditions[i][j].end.y;
            float segLength = sqrt(dist_x * dist_x + dist_y * dist_y);
            pathLength += segLength;

            if(actualPathSteps_[i][j] - pathLength > 1.0)
            {
                //PC found
                float speedVal = pathLength / actualPathSteps_[i][j];

                for(int k = lastIndex; k <= j; k++)
                {
                    _relativeVelocityProfile[i][k] = speedVal;
                }

                lastIndex = j;
                pathLength = actualPathSteps_[i][j];
            }
        }
    }

    return true;
}

void VelocityCalculator::Init(int nr_of_paths)
{
    pathStep_.clear();
    pathStep_.resize(nr_of_paths, 0);
    actualPathSteps_.resize(nr_of_paths);
    lockedPath_.clear();
    lockedPath_.resize(nr_of_paths, false);
}

VelocityCalculator::VelocityCalculator(int nr_of_paths)
{
    Init(nr_of_paths);
}

bool VelocityCalculator::MovePath(std::vector< PathSegment >& _pathWithPreconditions, int _pathNr)
{
    lockedPath_[_pathNr] = true;

    if(_pathWithPreconditions.size() !=  actualPathSteps_[_pathNr].size())
        actualPathSteps_[_pathNr].resize(_pathWithPreconditions.size());


    for(int i = pathStep_[_pathNr]; i < _pathWithPreconditions.size(); i++)
    {
        float dist_x = _pathWithPreconditions[i].start.x - _pathWithPreconditions[i].end.x;
        float dist_y = _pathWithPreconditions[i].start.y - _pathWithPreconditions[i].end.y;
        float length = sqrt(dist_x * dist_x + dist_y * dist_y);


        if(i == 0)
            actualPathSteps_[_pathNr][i] = length;
        else
            actualPathSteps_[_pathNr][i] = actualPathSteps_[_pathNr][i - 1] + length;


        for(auto & pc : _pathWithPreconditions[i].preconditions)
        {
            if(!MovePathUntil(paths_[pc.robot], pc.robot, pc.stepCondition + 1))
                return false;

            actualPathSteps_[_pathNr][i] = std::max<float>(actualPathSteps_[_pathNr][i], actualPathSteps_[pc.robot][pc.stepCondition + 1]);
        }

        pathStep_[_pathNr] = i;
    }

    lockedPath_[_pathNr] = false;
    return true;
}

bool VelocityCalculator::MovePathUntil(std::vector< PathSegment >& _pathWithPreconditions, int _pathNr, int _stepNr)
{
    if(lockedPath_[_pathNr] && _stepNr > pathStep_[_pathNr])
        return false;

    if(_pathWithPreconditions.size() !=  actualPathSteps_[_pathNr].size())
        actualPathSteps_[_pathNr].resize(_pathWithPreconditions.size());

    lockedPath_[_pathNr] = true;

    for(int i = pathStep_[_pathNr]; i < _stepNr; i++)
    {
        float dist_x = _pathWithPreconditions[i].start.x - _pathWithPreconditions[i].end.x;
        float dist_y = _pathWithPreconditions[i].start.y - _pathWithPreconditions[i].end.y;
        float length = sqrt(dist_x * dist_x + dist_y * dist_y);

        if(i == 0)
            actualPathSteps_[_pathNr][i] = length;
        else
            actualPathSteps_[_pathNr][i] = actualPathSteps_[_pathNr][i - 1] + length;


        for(auto & pc : _pathWithPreconditions[i].preconditions)
        {
            if(!MovePathUntil(paths_[pc.robot], pc.robot, pc.stepCondition + 1))
                return false;

            actualPathSteps_[_pathNr][i] = std::max<float>(actualPathSteps_[_pathNr][i], actualPathSteps_[pc.robot][pc.stepCondition + 1]);
        }

        pathStep_[_pathNr] = i;
    }

    lockedPath_[_pathNr] = false;
    return true;

}


