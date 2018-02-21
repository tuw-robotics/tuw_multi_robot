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

#include <tuw_global_planner/backtracking_avoid_resolution.h>

std::vector< std::shared_ptr< Segment > > BacktrackingAvoidResolution::resolve(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, std::shared_ptr< Segment > _end, int _collision, int _robot_radius)
{
    std::vector< std::shared_ptr< Segment > > retVals;

    float newPot = path_querry_->findPotentialUntilRobotOnSegment(_collision, _next);       //Margin of 2

    if(newPot == -1 || newPot == -2)    //Infinity
        return retVals;


    avoidStart(_current, _next, newPot, _robot_radius, retVals);
    avoidEnd(_current, _next, newPot, _robot_radius, retVals, _collision);

    trackBack(_current, _next, _end, _collision, _robot_radius, newPot, retVals);
    avoid(_current, _next, _end, newPot, _robot_radius, retVals);

    if(_current->planning.WaitSeg)
    {
        moveSegment(_current->planning.BacktrackingPredecessor, _current, _robot_radius, _collision, retVals);
    }

    return retVals;
}

void BacktrackingAvoidResolution::reset()
{
    avoidedSegments_.clear();
    
    for(int i = 0; i < createdSegmements_.size(); i++)
    {
        createdSegmements_[i]->clear();
    }
    createdSegmements_.clear();
}


void BacktrackingAvoidResolution::trackBack(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, std::shared_ptr< Segment > _end, int _collision, int _robot_radius, float _newPot, std::vector< std::shared_ptr< Segment > >& retVals)
{
    _next->planning.Potential = -1;
    _next->planning.Collision = -1;

    std::shared_ptr<Segment> newNext;
    std::shared_ptr<Segment> newCurr;

    //If no Backtracing possible return
    if(_current->planning.BacktrackingPredecessor->getIndex() == _current->getIndex())  //TODO
    {
        newNext = _next;
        newNext->planning.Potential = -1;
        newNext->planning.Collision = -1;

        newCurr = std::make_shared<Segment> ((*_current));
        newCurr->planning.Potential = _newPot + 2 * timeoverlap_;
        newCurr->planning.Collision = _collision;


        newCurr->planning.BacktrackingSuccessor = newNext;
        newNext->planning.BacktrackingSuccessor = nullptr;
        createdSegmements_.push_back(newCurr);
    }
    else
    {
        newNext = std::make_shared<Segment> ((*_current));
        newNext->planning.Potential = -1;
        newNext->planning.Collision = -1;

        newCurr = std::make_shared<Segment> (* (_current->planning.BacktrackingPredecessor));
        newCurr->planning.Potential = _newPot + 2 * timeoverlap_;
        newCurr->planning.Collision = _collision;


        newCurr->planning.BacktrackingSuccessor = newNext;
        newNext->planning.BacktrackingSuccessor = _next;
        
        
        createdSegmements_.push_back(newNext);
        createdSegmements_.push_back(newCurr);
    }

    if(!path_querry_->checkSegment(newCurr, (_current->planning.BacktrackingPredecessor)->planning.Potential - timeoverlap_,  newCurr->planning.Potential + timeoverlap_, _robot_radius, _collision))
    {
        if(!(_collision == -1 || _current->planning.BacktrackingPredecessor->getIndex() == _current->getIndex()))
        {
            auto vals = resolve(newCurr, newNext, _end, _collision, _robot_radius);
            retVals.insert(retVals.end(), vals.begin(), vals.end());
        }

    }
    else
    {
        retVals.push_back(newCurr);
    }
}



void BacktrackingAvoidResolution::avoid(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, std::shared_ptr< Segment > _end, float _newPot, int _robot_radius, std::vector< std::shared_ptr< Segment > >& retVals)
{
    Neighbours crossing = _current->getPredecessors();

    if(!crossing.contains(_next))
        crossing = _current->getSuccessors();


    if(crossing.contains(_next) && crossing.isCrossing())
    {
        for(auto seg_it = crossing.cbegin(); seg_it != crossing.cend(); seg_it++)
        {
            if((*seg_it)->getIndex() != _next->getIndex())
            {
                std::shared_ptr<Segment> newCurrent = std::make_shared<Segment>(*_current); 
                createdSegmements_.push_back(newCurrent);               
                newCurrent->planning.Potential = _current->planning.BacktrackingPredecessor->planning.Potential + pCalc_->CalculatePotential(_current);

                std::shared_ptr<Segment> cross_next = std::make_shared<Segment> (*(*seg_it));
                createdSegmements_.push_back(cross_next);
                cross_next->planning.BacktrackingPredecessor = newCurrent;
                cross_next->planning.Potential = std::max<float>(_newPot, newCurrent->planning.Potential + pCalc_->CalculatePotential(cross_next));
                cross_next->planning.Collision = -1;
                cross_next->planning.BacktrackingSuccessor = _next;
                cross_next->planning.WaitSeg = true;

                int coll = -1;

                if(path_querry_->checkSegment(cross_next, newCurrent->planning.Potential - timeoverlap_,  cross_next->planning.Potential + timeoverlap_, _robot_radius, coll))
                {
                    //Will not happen
                    retVals.push_back(cross_next);
                }
                else if(coll != -1)
                {
                    moveSegment(newCurrent, cross_next, _robot_radius, coll, retVals);
                }
            }
        }
    }
}

void  BacktrackingAvoidResolution::moveSegment(std::shared_ptr< Segment > _last,  std::shared_ptr< Segment > _waitSegment, int _robot_radius, int _coll, std::vector< std::shared_ptr< Segment > >& retVals)
{
    float newPot = path_querry_->findPotentialUntilRobotOnSegment(_coll, _waitSegment);       //Margin of 2

    Neighbours crossingW = _waitSegment->getPredecessors();

    if(crossingW.contains(_last))
        crossingW = _waitSegment->getSuccessors();


    if(!crossingW.contains(_last))
    {
        for(auto seg_w_it = crossingW.cbegin(); seg_w_it != crossingW.cend(); seg_w_it++)
        {
            int coll = -1;

            if(path_querry_->checkSegment(_waitSegment, _last->planning.Potential - timeoverlap_,  _last->planning.Potential + pCalc_->CalculatePotential(_waitSegment) + timeoverlap_, _robot_radius, coll))
            {
                //We can move in this direction without collision
                std::shared_ptr<Segment> moveToSegment = std::make_shared<Segment>(*_waitSegment);
                createdSegmements_.push_back(moveToSegment);

                moveToSegment->planning.Potential = _last->planning.Potential + pCalc_->CalculatePotential(moveToSegment) + timeoverlap_;  //Copy wait seg
                moveToSegment->planning.WaitSeg = false;

                std::shared_ptr<Segment> moveBackSegment = std::make_shared<Segment>(*moveToSegment);
                createdSegmements_.push_back(moveBackSegment);
                moveBackSegment->planning.Potential = -1;
                moveBackSegment->planning.Collision = -1;
                moveBackSegment->planning.WaitSeg = false;



                //Add new Wait segment
                float pot = std::max<float>(moveToSegment->planning.Potential + pCalc_->CalculatePotential(*seg_w_it) + timeoverlap_, newPot + 2 * timeoverlap_);

                std::shared_ptr<Segment> waitSegmentNew = std::make_shared<Segment>(*(*seg_w_it));
                createdSegmements_.push_back(waitSegmentNew);
                waitSegmentNew->planning.Potential = pot;
                waitSegmentNew->planning.Collision = -1;
                waitSegmentNew->planning.WaitSeg = true;

                //Set successor and predecessor

                moveBackSegment->planning.BacktrackingSuccessor = moveToSegment->planning.BacktrackingSuccessor;
                moveToSegment->planning.BacktrackingSuccessor = waitSegmentNew;
                moveBackSegment->planning.BacktrackingPredecessor = waitSegmentNew;
                waitSegmentNew->planning.BacktrackingPredecessor = moveToSegment;
                waitSegmentNew->planning.BacktrackingSuccessor = moveBackSegment;


                if(path_querry_->checkSegment(waitSegmentNew, moveToSegment->planning.Potential,  pot + timeoverlap_, _robot_radius, coll))
                {
                    retVals.push_back(waitSegmentNew);
                }
                else if(coll != -1)
                {
                    moveSegment(moveToSegment, waitSegmentNew, _robot_radius, coll, retVals);
                }
            }
            else if(coll != -1 && coll != _coll)
            {
                //TODO wait and be faster as first robot
            }

        }
    }
}

void BacktrackingAvoidResolution::avoidStart(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, float _newPot, int _robot_radius, std::vector< std::shared_ptr< Segment > >& retVals)
{
    if(!(_current->getIndex() == path_querry_->getStart()->getIndex()))
        return;

    Neighbours crossing = _current->getPredecessors();

    if(crossing.contains(_next))
        crossing = _current->getSuccessors();

    if(!crossing.contains(_next))
    {
        for(auto seg_it = crossing.cbegin(); seg_it != crossing.cend(); seg_it++)
        {
            if(std::find(avoidedSegments_.begin(), avoidedSegments_.end(), (*seg_it)->getIndex()) != avoidedSegments_.end())
                return;
            else
                avoidedSegments_.push_back((*seg_it)->getIndex());

            std::shared_ptr<Segment> newCurrent = std::make_shared<Segment>(*_current);
            createdSegmements_.push_back(newCurrent);
            newCurrent->planning.Potential = pCalc_->CalculatePotential(newCurrent);
            std::shared_ptr<Segment> newCurrentBack = std::make_shared<Segment>(*_current);
            createdSegmements_.push_back(newCurrentBack);
            newCurrentBack->planning.BacktrackingSuccessor = std::make_shared<Segment>(*_next);
            createdSegmements_.push_back(newCurrentBack->planning.BacktrackingSuccessor);
            newCurrentBack->planning.Potential = -1;


            std::shared_ptr<Segment> cross_next = std::make_shared<Segment> (*(*seg_it));
            createdSegmements_.push_back(cross_next);
            cross_next->planning.BacktrackingPredecessor = newCurrent;
            cross_next->planning.Potential = std::max<float>(_newPot, pCalc_->CalculatePotential(_current) + pCalc_->CalculatePotential(cross_next));
            cross_next->planning.Collision = -1;
            cross_next->planning.BacktrackingSuccessor = newCurrentBack;
            cross_next->planning.WaitSeg = true;

            int coll = -1;

            if(path_querry_->checkSegment(cross_next, pCalc_->CalculatePotential(_current) - timeoverlap_,  cross_next->planning.Potential + timeoverlap_, _robot_radius, coll))
            {
                retVals.push_back(cross_next);
            }
            else if(coll != -1)
            {
                moveSegment(newCurrent, cross_next, _robot_radius, coll, retVals);
            }

        }
    }
}


void BacktrackingAvoidResolution::avoidEnd(std::shared_ptr< Segment > _current, std::shared_ptr< Segment > _next, float _newPot, int _robot_radius, std::vector< std::shared_ptr< Segment > >& retVals, int _collision)
{
    if(!(_next->getIndex() == path_querry_->getEnd()->getIndex()))
        return;

    Neighbours crossing = _next->getPredecessors();

    if(crossing.contains(_current))
        crossing = _next->getSuccessors();


    if(!crossing.contains(_current))
    {
        int coll;

        if(path_querry_->checkSegment(_next, _current->planning.Potential - timeoverlap_,  _current->planning.Potential + pCalc_->CalculatePotential(_next) + timeoverlap_, _robot_radius, coll, true)) //TODO ERROR IS GOAL
        {
            _next->planning.Potential = _current->planning.Potential + pCalc_->CalculatePotential(_next);
            _next->planning.BacktrackingPredecessor = _current;
            _next->planning.Collision = _collision;

            for(auto seg_it = crossing.cbegin(); seg_it != crossing.cend(); seg_it++)
            {
                if(std::find(avoidedSegments_.begin(), avoidedSegments_.end(), (*seg_it)->getIndex()) != avoidedSegments_.end())
                    return;
                else
                    avoidedSegments_.push_back((*seg_it)->getIndex());

                std::shared_ptr<Segment> newCurrent = std::make_shared<Segment>(*_next);
                createdSegmements_.push_back(newCurrent);
                newCurrent->planning.Potential = _current->planning.BacktrackingPredecessor->planning.Potential + pCalc_->CalculatePotential(_current);

                std::shared_ptr<Segment> cross_next = std::make_shared<Segment> (*(*seg_it));
                createdSegmements_.push_back(cross_next);
                cross_next->planning.BacktrackingPredecessor = newCurrent;
                cross_next->planning.Potential = std::max<float>(_newPot, newCurrent->planning.Potential + pCalc_->CalculatePotential(cross_next));
                cross_next->planning.Collision = -1;
                cross_next->planning.BacktrackingSuccessor = _next;
                cross_next->planning.WaitSeg = true;

                int coll = -1;

                if(path_querry_->checkSegment(cross_next, newCurrent->planning.Potential - timeoverlap_,  cross_next->planning.Potential + timeoverlap_, _robot_radius, coll))
                {
                    retVals.push_back(cross_next);
                }
                else if(coll != -1)
                {
                    moveSegment(newCurrent, cross_next, _robot_radius, coll, retVals);
                }

            }
        }
        else if(coll != _collision)
        {
            //TODO ...
        }

    }
}
