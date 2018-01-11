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

#include <tuw_global_planner/traceback.h>
#include <ros/ros.h>

bool Traceback::getPath(std::shared_ptr< Segment > _startSeg, std::shared_ptr< Segment > _endSeg, std::vector< std::shared_ptr< Segment > >& _path)
{
    std::shared_ptr<Segment> current = _endSeg;
    current->planning.Direction = Segment::start_to_end;        //set any direction to not segfault
    _path.push_back(std::make_shared<Segment>(*current));       //Copy segments to keep values for planning unique...


    while(current->getIndex() != _startSeg->getIndex() || (current->planning.BacktrackingPredecessor->getIndex() != _startSeg->getIndex() && current->planning.BacktrackingPredecessor->getIndex() != -1))
    {
        std::shared_ptr<Segment> pred = std::make_shared<Segment>(*current->planning.BacktrackingPredecessor);

        if(pred->getIndex()  == current->getIndex() && current->getIndex() != _startSeg->getIndex())
            return false;


        if(pred->isSuccessor(current))
            pred->planning.Direction = Segment::end_to_start;   //-1
        else
            pred->planning.Direction = Segment::start_to_end;   //1

        _path.push_back(pred);

        current = pred;
    }

    return true;
}
