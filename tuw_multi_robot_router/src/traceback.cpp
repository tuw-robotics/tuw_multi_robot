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

#include <tuw_global_router/traceback.h>
#include <ros/ros.h>

namespace multi_robot_router
{

bool Traceback::getPath(const Vertex &_startSeg, const Vertex &_endSeg, std::vector<RouteVertex> &_path) const
{
    const Vertex *current = &_endSeg;
    const Vertex *predecessor = current->predecessor_;

    //Set moving direction
    _path.emplace_back(*current);
    _path.back().direction = RouteVertex::path_direction::end_to_start;

    if (predecessor != NULL && isSuccessor(current, predecessor))
        _path.back().direction = RouteVertex::path_direction::start_to_end;

    while (current->predecessor_ != NULL)
    {
        const Vertex *pred = current->predecessor_;

        _path.emplace_back(*pred);

        if (isSuccessor(pred, current))
            _path.back().direction = RouteVertex::path_direction::end_to_start; //-1
        else
            _path.back().direction = RouteVertex::path_direction::start_to_end; //1

        current = pred;
    }

    if (_path.back().getSegment().getSegmentId() != _startSeg.getSegment().getSegmentId())
        return false;

    return true;
}

bool Traceback::isSuccessor(const Vertex *_vertex, const Vertex *_succ) const
{
    const std::vector<std::reference_wrapper<Vertex>> succ = _vertex->getPlanningSuccessors();

    for (const Vertex &s : succ)
    {
        if (s.getSegment().getSegmentId() == _succ->getSegment().getSegmentId())
            return true;
    }

    return false;
}

} // namespace multi_robot_router
