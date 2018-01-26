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

#include <tuw_global_planner/point_expander.h>

void PointExpander::initialize(const grid_map::GridMap& _map)
{
    nx_ = _map.getSize()[0];
    ny_ = _map.getSize()[1];
    ns_ = nx_ * ny_;

    distance_field_.reset(new float[nx_ * ny_]);
    voronoi_graph_.reset(new int8_t[nx_ * ny_]);
    global_map_.reset(new int8_t[nx_ * ny_]);

    float* distance_field = distance_field_.get();
    int8_t* voronoi_graph = voronoi_graph_.get();
    int8_t* global_map = global_map_.get();

    getMaps(distance_field, voronoi_graph, global_map, _map);
}


void PointExpander::addPotentialExpansionCandidate(PointExpander::Index _current, int _next_x, int _next_y, float* _potential)
{
    float potentialPrev = _potential[_current.i];
    Index next = _current.offsetDist(_next_x, _next_y, nx_, ny_);

    //Check Boundries
    if(next.i < 0 || next.i > ns_)
        return;

    //Dont update allready found potentials
    if(_potential[next.i] < POT_HIGH)
        return;

    if(global_map_[next.i] > 0)
        return;


    float dist = 0;
    float pot = potentialPrev + neutral_cost_;
    float weight = pot;                         //Dijkstra

    _potential[next.i] = pot;

    queue_.push(Index(next.i, weight, dist, pot));

}

PointExpander::Index PointExpander::findGoal(PointExpander::Index _start, int _cycles, float* _potential, const std::map<int, Index> &_goals, int _optimizationSteps, int &segIdx)
{
    std::fill(_potential, _potential + ns_, POT_HIGH);
    int cycle = 0;

    int _noGoalPoses = _optimizationSteps + 1;

    Index current(_start.i, 0, 0, 0);
    _potential[current.i] = 0;

    //clear the queue
    clearpq(queue_);
    queue_.push(current);

    Index currentGoal(-1, -1, -1, -1);
	segIdx = -1;

    while(!queue_.empty() && _noGoalPoses > 0 && cycle < _cycles)       
    {
        if(queue_.empty())
            return Index(-1, -1, -1, -1);

        current = queue_.top();
        queue_.pop();


		int segmentIndex = -1;
        if(isGoal(current, _goals, segmentIndex))
        {
            if(currentGoal.i == -1)
            {
                currentGoal = current;
				segIdx = segmentIndex;
            }
            else
            {
                if(_potential[current.i] < _potential[currentGoal.i] + current.distance(currentGoal, nx_))
                {
                    currentGoal = current;
					segIdx = segmentIndex;
                }
            }

            _noGoalPoses--;

            if(_noGoalPoses == 0)
                return currentGoal;
        }


        addPotentialExpansionCandidate(current, 1, 0, _potential);
        addPotentialExpansionCandidate(current, 0, 1, _potential);
        addPotentialExpansionCandidate(current, -1, 0, _potential);
        addPotentialExpansionCandidate(current, 0, -1, _potential);

        cycle++;
    }

    if(cycle >= _cycles)
        return Index(-1, -1, -1, -1);
    else
        return _start;
}

bool PointExpander::isGoal(Index _p, const std::map<int, Index> &_goals, int &segIdx)
{
	segIdx = -1;
    for(const std::pair<int, Index> & g : _goals)
    {
        if(_p.i == g.second.i)
		{
			segIdx = g.first;
            return true;
		}
    }

    return false;
}


bool PointExpander::findGoalOnMap(const Point &_start, int _cycles, float* _potential, const std::map<int, Point> &_goals, int _optimizationSteps, Point &_foundPoint, int &_segIdx)
{
    Index startPoint((int)_start[0], (int)_start[1], nx_, 0, 0, 0);
    Index foundPoint(-1, -1, -1, -1);

    std::map<int, Index> goals;

    for(const std::pair<int,Point> & g : _goals)
    {
		Index idx((int)g.second[0], (int)g.second[1], nx_, 0, 0, 0);
		std::pair<int,Index> i(g.first, idx);
        goals.insert(i);
    }

    foundPoint = findGoal(startPoint, _cycles, _potential, goals, _optimizationSteps, _segIdx);
    _foundPoint[0] = ((int)foundPoint.getX(nx_));
    _foundPoint[1] = ((int)foundPoint.getY(nx_));


    return (foundPoint.i >= 0);
}

void PointExpander::getMaps(float* _distance_field, int8_t* _voronoi_graph, int8_t* _global_map, const grid_map::GridMap& _voronoi_map)
{
    auto& distfield = _voronoi_map.get("distfield");
    auto& voronoi = _voronoi_map.get("voronoi");
    auto& map = _voronoi_map.get("map");


    for(grid_map::GridMapIterator iterator(_voronoi_map); !iterator.isPastEnd(); ++iterator)
    {
        const grid_map::Index mapIndex = iterator.getUnwrappedIndex();
        int arrayIndex = (ny_ - 1 - mapIndex[1]) * nx_ + (nx_ - 1 - mapIndex[0]);
        _distance_field[arrayIndex] = distfield(iterator.getLinearIndex());
        _voronoi_graph[arrayIndex] = voronoi(iterator.getLinearIndex());
        _global_map[arrayIndex] = map(iterator.getLinearIndex());
    }
}
