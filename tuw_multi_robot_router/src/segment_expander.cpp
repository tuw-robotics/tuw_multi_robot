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

//TODO give potential...

#include <tuw_global_planner/segment_expander.h>
#include <tuw_global_planner/path_coordinator.h>
#include <grid_map_ros/grid_map_ros.hpp>
#define TIME_OVERLAP 1

SegmentExpander::SegmentExpander(std::shared_ptr<Heuristic> _h, std::shared_ptr<PotentialCalculator> _pCalc, std::shared_ptr<Path_Coordinator> _pathCoordinator, std::shared_ptr<CollisionResolution> _cr) : hx_(_h), pCalc_(_pCalc), path_querry_(_pathCoordinator), collision_resolution_(_cr)
{ }

void SegmentExpander::addVoronoiExpansoionCandidate(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, std::shared_ptr< Segment > _end)
{
    if(_current->planning.Potential == -1)
        return;

    if(_next->planning.Potential >= 0)
        return;


    int collision;

    if(!path_querry_->checkSegment(_next, _current->planning.Potential - TIME_OVERLAP, _current->planning.Potential + pCalc_->CalculatePotential(_next) + TIME_OVERLAP, radius_, collision))
    {
        if(collision != -1)
        {
            std::vector<std::shared_ptr<Segment>> resolutions = collision_resolution_->resolve(_current, _next, _end, collision, radius_);

            if(resolutions.size() == 0)
            {
                if(collisions_robots_.size() <= collision)
                {
                    collisions_robots_.resize(collision + 1, 0);
                }

                collisions_robots_[collision] += 100;
            }
            else
            {
                if(collisions_robots_.size() <= collision)
                {
                    collisions_robots_.resize(collision + 1, 0);
                }

                collisions_robots_[collision] ++;
            }


            for(auto & res : resolutions)
            {
                float h = hx_->calcHeuristic(res, _end);
                res->planning.Weight =  res->planning.Potential + h;

                if(res->getIndex() == _end->getIndex())                 //Should not happen but safety first
                    res->planning.Weight = 0;

                seg_queue_.push(res);
            }
        }

        return;
    }


    float pot = _current->planning.Potential + pCalc_->CalculatePotential(_next);
    float h = hx_->calcHeuristic(_next, _end);
    float weight = pot + h;

    _next->planning.Weight = weight;
    _next->planning.Potential = pot;

    if(!(_next->getIndex() != _end->getIndex()))
        _next->planning.Weight = 0;

    _next->planning.BacktrackingPredecessor = _current;
    _next->planning.Collision = -1;

    seg_queue_.push(_next);
}

bool SegmentExpander::calculatePotentials(std::shared_ptr< Segment > _start, std::shared_ptr< Segment > _end, std::vector< std::shared_ptr< Segment > > _graph, float _radius)
{
    collisions_robots_.clear();
    collision_resolution_->reset();
    radius_ = _radius;

    for(auto & seg : (_graph))
    {
        Segment::astar_planning planEmpty;
        seg->planning = planEmpty;
    }

    std::shared_ptr<Segment> foundEnd = expandVoronoi(_start, _end, _graph.size() * 50);

    //Save actual planning status from parallel ends :D
    if(_end != foundEnd)
        _end->planning = foundEnd->planning;

    for(int i = 0; i < collisions_robots_.size(); i++)
    {
        path_querry_->updateNrOfCollisions(i, collisions_robots_[i]);
    }

    return (foundEnd->getIndex() == _end->getIndex());
}

void SegmentExpander::resolveStartCollision(std::shared_ptr< Segment > _start, std::shared_ptr< Segment > _end)
{
    int collision = -1;
    _start->planning.Collision = -1;
    _start->planning.BacktrackingPredecessor = NULL;
    _start->planning.Potential = -1;

    clearpq(seg_queue_);

    if(_start->getIndex() == _end->getIndex() && !path_querry_->checkSegment(_start, 0, -1, radius_, collision) && collision != -1)
    {
        // If we cant wait on the start point add a intermediate layer
        //copy goal
        std::shared_ptr<Segment> start = std::make_shared<Segment>(*_start);
        start->planning.BacktrackingPredecessor = start;
        start->planning.Potential = pCalc_->CalculatePotential(start);         //For having a high startpotential
        start->planning.Collision = -1;

        Neighbours n_succ = start->getSuccessors();

        for(auto it = n_succ.cbegin(); it != n_succ.cend(); it++)
        {
            if(path_querry_->checkSegment((*it), start->planning.Potential - TIME_OVERLAP, start->planning.Potential + pCalc_->CalculatePotential((*it)) + TIME_OVERLAP, radius_, collision))
            {
                float pot = start->planning.Potential + pCalc_->CalculatePotential((*it));
                float h = hx_->calcHeuristic((*it), _end);
                float weight = pot + h;

                (*it)->planning.Weight = weight;
                (*it)->planning.Potential = pot;
                (*it)->planning.BacktrackingPredecessor = start;
                (*it)->planning.BacktrackingSuccessor = _start;
                (*it)->planning.Collision = -1;

                seg_queue_.push((*it));
            }
        }


        Neighbours n_pred = start->getPredecessors();

        for(auto it = n_pred.cbegin(); it != n_pred.cend(); it++)
        {
            if(path_querry_->checkSegment((*it), start->planning.Potential - TIME_OVERLAP, start->planning.Potential + pCalc_->CalculatePotential((*it)) + TIME_OVERLAP, radius_, collision))
            {
                float pot = start->planning.Potential + pCalc_->CalculatePotential((*it));
                float h = hx_->calcHeuristic((*it), _end);
                float weight = pot + h;

                (*it)->planning.Weight = weight;
                (*it)->planning.Potential = pot;
                (*it)->planning.BacktrackingPredecessor = start;
                (*it)->planning.BacktrackingSuccessor = _start;
                (*it)->planning.Collision = -1;

                seg_queue_.push((*it));
            }
        }

    }
    else
    {
        _start->planning.Collision = -1;
        _start->planning.BacktrackingPredecessor = _start;
        _start->planning.Potential = pCalc_->CalculatePotential(_start);



        seg_queue_.push(_start);
    }
}


std::shared_ptr< Segment > SegmentExpander::expandVoronoi(std::shared_ptr< Segment > _start, std::shared_ptr< Segment > _end, int _cycles)
{
    int cycle = 0;                                        //For having a high startpotential
    resolveStartCollision(_start, _end);

    std::shared_ptr<Segment> current = std::make_shared<Segment>();

    while(!(seg_queue_.empty())  && (cycle < _cycles) && (_end->getIndex() != current->getIndex()))
    {
        if(seg_queue_.empty())
        {
            return std::shared_ptr<Segment> (new Segment());
        }

        current = seg_queue_.top();
        seg_queue_.pop();

        if(_end->getIndex() == current->getIndex() && current->planning.BacktrackingSuccessor.use_count() == 0)
        {
            return current;
        }


        if(current->planning.BacktrackingSuccessor.use_count() != 0)
        {
            addVoronoiExpansoionCandidate(current, current->planning.BacktrackingSuccessor, _end);
        }
        else
        {
            Neighbours n_succ = current->getSuccessors();

            if(!n_succ.contains(current->planning.BacktrackingPredecessor))
            {
                for(auto it = n_succ.cbegin(); it != n_succ.cend(); it++)
                {
                    if((*it)->isPredecessor(current))
                        (*it)->planning.Direction = Segment::end_to_start;
                    else
                        (*it)->planning.Direction = Segment::start_to_end;


                    addVoronoiExpansoionCandidate(current, (*it), _end);
                }
            }

            Neighbours n_pred = current->getPredecessors();

            if(!n_pred.contains(current->planning.BacktrackingPredecessor))
            {
                for(auto it = n_pred.cbegin(); it != n_pred.cend(); it++)
                {
                    if((*it)->isPredecessor(current))
                        (*it)->planning.Direction = Segment::end_to_start;
                    else
                        (*it)->planning.Direction = Segment::start_to_end;


                    addVoronoiExpansoionCandidate(current, (*it), _end);
                }
            }
        }

        cycle++;
    }

    if((cycle >= _cycles))
        ROS_INFO("TO LESS CYCLES");

    if(current->getIndex() != _end->getIndex())
    {
        return std::shared_ptr<Segment> (new Segment());
    }

    return current;
}



