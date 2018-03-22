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
#include <tuw_global_planner/route_coordinator.h>
#include <iostream>
#define TIME_OVERLAP 1
#define SEARCH_DEPTH 20 //TODO settings

SegmentExpander::SegmentExpander(const Heuristic &_h, const PotentialCalculator &_pCalc)  
{
    hx_ = std::make_unique<Heuristic>(_h);
    pCalc_ = std::make_unique<PotentialCalculator>(_pCalc);
    collision_resolution_ = std::make_unique<BacktrackingResolution>(TIME_OVERLAP);
}

void SegmentExpander::reset()
{
    collisions_robots_.clear();
    //TODO Expansion segments Not sure if new
}

void SegmentExpander::addVoronoiExpansoionCandidate(Vertex &_current, Vertex &_next, Vertex &_end)
{
    if(_current.potential == -1)
        return;

    if(_next.potential >= 0)
        return;


    int32_t collision; 
    if(!route_querry_->checkSegment(_next, _current.potential - TIME_OVERLAP, _current.potential + pCalc_->CalculatePotential(_next) + TIME_OVERLAP, diameter_, collision))
    {
        if(collision != -1)
        {
            std::vector<std::reference_wrapper<Vertex>> resolutions = collision_resolution_->resolve(_current, _next, collision);
            
            for(Vertex& res : resolutions)
            {
                float h = hx_->calcHeuristic(res, _end);
                res.weight =  res.potential + h;

                if(res.getSegment().getSegmentId() == _end.getSegment().getSegmentId())                 //Should not happen but safety first :D
                {
                    res.weight = 0;
                }

                seg_queue_.push(&res);
            }
        }

        return;
    }


    float pot = _current.potential + pCalc_->CalculatePotential(_next);
    float h = hx_->calcHeuristic(_next, _end);
    float weight = pot + h;

    _next.weight = weight;
    _next.potential = pot;
    
    if(_next.getSegment().getSegmentId() == _end.getSegment().getSegmentId())
        _next.weight = 0;

    _next.predecessor_ = &_current;
    _next.collision = -1;

    seg_queue_.push(&_next);
}

bool SegmentExpander::calculatePotentials(const RouteCoordinator *_p, Vertex & _start, Vertex &_end, const uint32_t _maxIterations, const uint32_t _diameter)
{
    route_querry_ = _p;

    collisions_robots_.clear();
    collision_resolution_->resetSession(_p, pCalc_.get(), _diameter);
    diameter_ = _diameter;

    Vertex *foundEnd = expandVoronoi(_start, _end, _maxIterations);

    if(foundEnd == NULL)
        return false;

    //Save actual planning status from parallel ends :D
    if(&_end != foundEnd)
        _end.updateVertex(*foundEnd);

    return (foundEnd->getSegment().getSegmentId() == _end.getSegment().getSegmentId());
}

void SegmentExpander::resolveStartCollision(Vertex &_start, Vertex &_end)
{
//     int collision = -1;
//     _start.collision = -1;
//     _start.predecessor_ = NULL;
//     _start.potential = -1;
//
//       clearpq(seg_queue_);
//
//     if(_start.getSegment().getSegmentId() == _end.getSegment().getSegmentId() && !path_querry_->checkSegment(_start, 0, -1, radius_, collision) && collision != -1)
//     {
//         // If we cant wait on the start point add a intermediate layer
//         //copy goal
//         Vertex start(_start);                       //TODO TODO return reference
//         start.predecessor_ = start;
//         start->planning.Potential = pCalc_->CalculatePotential(start);         //For having a high startpotential
//         start->planning.Collision = -1;
//
//         Neighbours n_succ = start->getSuccessors();
//
//         for(auto it = n_succ.cbegin(); it != n_succ.cend(); it++)
//         {
//             if(path_querry_->checkSegment((*it), start->planning.Potential - TIME_OVERLAP, start->planning.Potential + pCalc_->CalculatePotential((*it)) + TIME_OVERLAP, radius_, collision))
//             {
//                 float pot = start->planning.Potential + pCalc_->CalculatePotential((*it));
//                 float h = hx_->calcHeuristic((*it), _end);
//                 float weight = pot + h;
//
//                 (*it)->planning.Weight = weight;
//                 (*it)->planning.Potential = pot;
//                 (*it)->planning.BacktrackingPredecessor = start;
//                 (*it)->planning.BacktrackingSuccessor = _start;
//                 (*it)->planning.Collision = -1;
//
//                 seg_queue_.push((*it));
//             }
//         }
//
//
//         Neighbours n_pred = start->getPredecessors();
//
//         for(auto it = n_pred.cbegin(); it != n_pred.cend(); it++)
//         {
//             if(path_querry_->checkSegment((*it), start->planning.Potential - TIME_OVERLAP, start->planning.Potential + pCalc_->CalculatePotential((*it)) + TIME_OVERLAP, radius_, collision))
//             {
//                 float pot = start->planning.Potential + pCalc_->CalculatePotential((*it));
//                 float h = hx_->calcHeuristic((*it), _end);
//                 float weight = pot + h;
//
//                 (*it)->planning.Weight = weight;
//                 (*it)->planning.Potential = pot;
//                 (*it)->planning.BacktrackingPredecessor = start;
//                 (*it)->planning.BacktrackingSuccessor = _start;
//                 (*it)->planning.Collision = -1;
//
//                 seg_queue_.push((*it));
//             }
//         }
//
//     }
//     else
    {
        _start.collision = -1;
        _start.predecessor_ = NULL;
        _start.potential = pCalc_->CalculatePotential(_start);
        clearpq(seg_queue_);
        int32_t collision = -1;
        if(route_querry_->checkSegment(_start, 0, _start.potential + TIME_OVERLAP, diameter_, collision))
        {
            seg_queue_.push(&_start);
        }
        else if(collision != -1)
        {
            collision_resolution_->saveCollision(collision);
        }
    }
}
bool SegmentExpander::containsVertex(const Vertex& _v, const std::vector< std::reference_wrapper< Vertex > >& _list) const
{
    for(const Vertex & v : _list)
    {
        if(_v.getSegment().getSegmentId() == v.getSegment().getSegmentId())
            return true;
    }

    return false;
}


void SegmentExpander::setSpeed(const float &_speed)
{
    pCalc_->SetMultiplier(_speed);
}

Vertex *SegmentExpander::expandVoronoi(Vertex &_start, Vertex &_end, const uint32_t _cycles)
{
    uint32_t cycle = 0;                                        //For having a high startpotential
    resolveStartCollision(_start, _end);

    Vertex *current = NULL;

    while(!(seg_queue_.empty())  && (cycle < _cycles) && (current == NULL || _end.getSegment().getSegmentId() != current->getSegment().getSegmentId()))
    {
        if(seg_queue_.empty())
            return NULL;

        current = seg_queue_.top();
        seg_queue_.pop();


        //The segment expander is forced to expand to the successor (generated by the collision resolution)
        if(current->successor_ != NULL) 
        {
            addVoronoiExpansoionCandidate(*current, *(current->successor_), _end);
        }
        else
        {
            const std::vector< std::reference_wrapper< Vertex > >& n_succ = current->getPlanningSuccessors();
            if(current->predecessor_ == NULL || !containsVertex(*(current->predecessor_), n_succ))
            {
                for(Vertex & v : n_succ)
                {
                    addVoronoiExpansoionCandidate(*current, v, _end);
                }
            }

            const std::vector< std::reference_wrapper< Vertex > >& n_pred = current->getPlanningPredecessors();

            if(current->predecessor_ == NULL || !containsVertex(*(current->predecessor_), n_pred))
            {
                for(Vertex & v : n_pred)
                {
                    addVoronoiExpansoionCandidate(*current, v, _end);
                }
            }
        }
        
        cycle++;
    }


    if(current == NULL || current->getSegment().getSegmentId() != _end.getSegment().getSegmentId())
        return NULL;

    return current;
}

const std::vector<uint32_t> &SegmentExpander::getRobotCollisions() const
{
    return collision_resolution_->getRobotCollisions();
}

