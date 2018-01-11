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

#include <tuw_global_planner/backtracking_resolution.h>

std::vector< std::shared_ptr< Segment > > BacktrackingResolution::resolve(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, std::shared_ptr< Segment > _end, int _collision, int _robot_radius)
{
    std::vector< std::shared_ptr< Segment > > retVals;

    float newPot = path_querry_->findPotentialUntilRobotOnSegment(_collision, _next);       //Margin of 2

    //ROS_INFO ("BT %f", newPot);

    if(newPot == -1)    //Infinity
        return retVals;

    _next->planning.Potential = -1;
    _next->planning.Collision = -1;

    std::shared_ptr<Segment> newNext;
    std::shared_ptr<Segment> newCurr;

    //If no Backtracing possible return
    if(_current->planning.BacktrackingPredecessor->getIndex() == _current->getIndex())
    {
        newNext = _next;
        newNext->planning.Potential = -1;
        newNext->planning.Collision = -1;

        newCurr = std::make_shared<Segment> ((*_current));
        newCurr->planning.Potential = newPot + 2 * timeoverlap_;
        newCurr->planning.Collision = _collision;


        newCurr->planning.BacktrackingSuccessor = newNext;
        newNext->planning.BacktrackingSuccessor = nullptr;
    }
    else
    {
        newNext = std::make_shared<Segment> ((*_current));
        newNext->planning.Potential = -1;
        newNext->planning.Collision = -1;

        newCurr = std::make_shared<Segment> (* (_current->planning.BacktrackingPredecessor));
        newCurr->planning.Potential = newPot + 2 * timeoverlap_;
        newCurr->planning.Collision = _collision;


        newCurr->planning.BacktrackingSuccessor = newNext;
        newNext->planning.BacktrackingSuccessor = _next;
    }


    if(!path_querry_->checkSegment(newCurr, (_current->planning.BacktrackingPredecessor)->planning.Potential - timeoverlap_,  newCurr->planning.Potential + timeoverlap_, _robot_radius, _collision))
    {
        if(_collision == -1 || _current->planning.BacktrackingPredecessor->getIndex() == _current->getIndex())
            return retVals;

        return resolve(newCurr, newNext, _end, _collision, _robot_radius);
    }
    else
    {
        retVals.push_back(newCurr);
    }

    return retVals;
}










