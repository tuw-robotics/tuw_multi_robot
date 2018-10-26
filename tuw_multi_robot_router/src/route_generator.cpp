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

#include <tuw_global_router/route_generator.h>

namespace multi_robot_router
{
std::vector<std::vector<Checkpoint>> RouteGenerator::generatePath(const std::vector<std::vector<RouteVertex>> &_paths, const RouteCoordinator &routeQuerry_) const
{
    std::vector<std::vector<Checkpoint>> generatedPaths;

    for (uint32_t i = 0; i < _paths.size(); i++)
    {
        std::vector<Checkpoint> generatedPath;

        for (uint32_t j = 0; j < _paths[i].size(); j++)
        {
            if (_paths[i][j].direction == RouteVertex::path_direction::none)
            {
                generatedPath.clear();
                return generatedPaths;
            }

            Checkpoint seg;
            seg = createElement(_paths[i][j]);

            addPreconditions(seg, _paths[i][j], i, _paths, routeQuerry_);
            generatedPath.push_back(seg);
        }

        generatedPaths.push_back(generatedPath);
    }

    return generatedPaths;
}

Checkpoint RouteGenerator::createElement(const RouteVertex &_element) const
{
    Checkpoint ps;

    if (_element.direction == RouteVertex::path_direction::start_to_end)
    {
        ps.segId = _element.getSegment().getSegmentId();
        ps.end[0] = _element.getSegment().getStart()[0];
        ps.end[1] = _element.getSegment().getStart()[1];
        ps.start[0] = _element.getSegment().getEnd()[0];
        ps.start[1] = _element.getSegment().getEnd()[1];

        float angle = atan2(ps.start[1] - ps.end[1], ps.start[0] - ps.end[0]);
        ps.start[2] = angle;
        ps.end[2] = angle;
    }
    else
    {
        ps.segId = _element.getSegment().getSegmentId();
        ps.end[0] = _element.getSegment().getEnd()[0];
        ps.end[1] = _element.getSegment().getEnd()[1];
        ps.start[0] = _element.getSegment().getStart()[0];
        ps.start[1] = _element.getSegment().getStart()[1];

        float angle = atan2(ps.start[1] - ps.end[1], ps.start[0] - ps.end[0]);
        ps.start[2] = angle;
        ps.end[2] = angle;
    }

    return ps;
}

void RouteGenerator::addPreconditions(Checkpoint &_segment, const RouteVertex &_segToFind, const uint32_t _pathNr, const std::vector<std::vector<RouteVertex>> &_paths, const RouteCoordinator &routeQuerry_) const
{
    std::vector<std::pair<uint32_t, float>> list = routeQuerry_.getListOfRobotsHigherPrioritizedRobots(_pathNr, _segToFind.getSegment().getSegmentId(), _segToFind.potential);
    _segment.preconditions.clear();

    for (const std::pair<uint32_t, float> &rob : list)
    {
        bool found = false;

        for (uint32_t i = 0; i < _paths[rob.first].size(); i++)
        {
            if (_paths[rob.first][i].potential >= rob.second)
            {
                Checkpoint::Precondition p;
                p.robotId = rob.first;
                p.stepCondition = i;
                _segment.updatePreconditions(p);

                found = true;
                break;
            }
        }
    }
}
} // namespace multi_robot_router
