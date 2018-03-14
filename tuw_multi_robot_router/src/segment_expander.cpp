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
#define TIME_OVERLAP 1

SegmentExpander::SegmentExpander(const Heuristic &_h, const PotentialCalculator &_pCalc)
{
    hx_ = std::make_unique<Heuristic>(_h);
    pCalc_ = std::make_unique<PotentialCalculator>(_pCalc);
}

void SegmentExpander::reset()
{
    collisions_robots_.clear();
    //TODO Expansion segments
}

void SegmentExpander::addVoronoiExpansoionCandidate(Vertex &_current, Vertex &_next, Vertex &_end)
{
    if(_current.potential == -1)
        return;

    if(_next.potential >= 0)
        return;


    int32_t collision;

    if(!path_querry_->checkSegment(_next, _current.potential - TIME_OVERLAP, _current.potential + pCalc_->CalculatePotential(_next) + TIME_OVERLAP, radius_, collision))
    {
        if(collision != -1)
        {
            //TODO HACK
            if(collisions_robots_.size() <= collision)
                collisions_robots_.resize(collision + 1, 0);
            collisions_robots_[collision]++;
          
//             std::vector<Vertex> resolutions;// = collision_resolution_->resolve(_current, _next, _end, collision, radius_);
//
//             if(resolutions.size() == 0)
//             {
//                 if(collisions_robots_.size() <= collision)
//                 {
//                     collisions_robots_.resize(collision + 1, 0);
//                 }
//
//                 collisions_robots_[collision] += 100;
//             }
//             else
//             {
//                 if(collisions_robots_.size() <= collision)
//                 {
//                     collisions_robots_.resize(collision + 1, 0);
//                 }
//
//                 collisions_robots_[collision] ++;
//             }
//
//
//             for(Vertex& res : resolutions)
//             {
//                 float h = hx_->calcHeuristic(res, _end);
//                 res.weight =  res.potential + h;
//
//                 if(res.getSegment().getSegmentId() == _end.getSegment().getSegmentId())                 //Should not happen but safety first
//                     res.weight = 0;
//
//                 seg_queue_.push(&res);
//             }
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

bool SegmentExpander::calculatePotentials(const RouteCoordinator *_p, Vertex & _start, Vertex &_end, std::vector<Vertex> &_graph, const uint32_t _radius)
{
    path_querry_ = _p;

    collisions_robots_.clear();
    //collision_resolution_->reset();
    radius_ = _radius;

    Vertex *foundEnd = expandVoronoi(_start, _end, _graph.size() * 20); //TODO Check Size ...

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
        seg_queue_.push(&_start);
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

        cycle++;
    }


    if(current == NULL || current->getSegment().getSegmentId() != _end.getSegment().getSegmentId())
        return NULL;

    return current;
}

const std::vector<uint32_t> &SegmentExpander::getRobotCollisions() const
{
    return collisions_robots_;
}

