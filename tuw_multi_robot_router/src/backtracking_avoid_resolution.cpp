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

BacktrackingAvoidResolution::BacktrackingAvoidResolution(uint32_t _timeoverlap) : timeoverlap_(_timeoverlap)
{
}

void BacktrackingAvoidResolution::resetSession(const RouteCoordinator *_route_querry, const PotentialCalculator *_pCalc, const uint32_t _robotDiameter)
{
    route_querry_ = _route_querry;
    pCalc_ = _pCalc;
    robotDiameter_ = _robotDiameter;
    generatedSubgraphs_.clear();
    encounteredCollisions_.clear();
    resolutionAttemp_ = 0;
}


void BacktrackingAvoidResolution::addCollision(const uint32_t robot)
{
    if(encounteredCollisions_.size() <= robot)
        encounteredCollisions_.resize(robot + 1, 0);

    encounteredCollisions_[robot]++;
}


std::vector<std::reference_wrapper<Vertex>> BacktrackingAvoidResolution::resolve(Vertex &_current, Vertex &_next, int32_t _collision)
{
    generatedSubgraphs_.emplace_back();
    foundSolutions_.clear();

    //Triggered when a robot blocks a vertex
    addCollision(_collision);

    //find the potential when the collision robots leaves the segment
    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(_collision, _next.getSegment().getSegmentId());

    //There is no solution if the colliding robot never leaves the segment.
    //Therefore, do nothing when leavePotential is smaller zero
    if(leavePotential >= 0)
    {
        trackBack(_current, _next, _collision, leavePotential);
    }

    resolutionAttemp_++;
    return foundSolutions_;
}





void BacktrackingAvoidResolution::trackBack(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //Free the next Vertex for new expansion
    _next.potential = -1;
    _next.collision = -1;

    Vertex *next_n;
    Vertex *current_n;

    //If backtracking is not possible (we are on the start vertex) try to wait there
    //Additionally Backtracking beond wait Segments is not allowed to be able to solve
    //multi robot scenarios (avoiding n robots in a row) 
    //Anyway backtracking beond wait Segments makes no sense, we will only find allready
    //found solutions...
    if(_current.predecessor_ == NULL || _current.isWaitSegment)
    {
        int32_t collision = -1;

        if(route_querry_->checkSegment(_current, 0,  _freePotential + 2 * timeoverlap_, robotDiameter_, collision))
        {
            _next.potential = -1;
            _next.collision = -1;
            //_next.successor_ = NULL; //Tell expander to expand the vertex normally


            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;
            current_n.successor_ = &_next;    //Tell expander to only expand to next

            foundSolutions_.push_back(current_n);
        }
        else
        {
            if(_collision != collision)
                addCollision(collision);

            float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, _next.getSegment().getSegmentId());
            //avoidStart(current_n, _next, collision, leavePotential);
        }
    }
    else    //we are somewhere on the path
    {
        int32_t collision = -1;
        bool vertexFree = route_querry_->checkSegment(*_current.predecessor_, _current.predecessor_->potential - timeoverlap_,  _freePotential + 2 * timeoverlap_, robotDiameter_, collision);

        if(vertexFree || collision != -1)
        {
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            next_n.potential = -1;
            next_n.collision = -1;

            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(*(_current.predecessor_)));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;

            //Set References
            current_n.successor_ = &next_n;             //Tell expander to only expand to "new next" (with new Potential)
            next_n.predecessor_ = &current_n;
            next_n.successor_ = &_next;         //Tell expander to only expand to next (we are not allowed to leave the backtracked path)

            if(vertexFree)
            {
                foundSolutions_.push_back(current_n);
            }
            else if(collision != -1)
            {
                if(collision != -1 && collision != _collision)
                    addCollision(collision);


                float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, next_n.getSegment().getSegmentId());
                trackBack(current_n, next_n, collision, leavePotential);
                avoid(current_n, next_n, collision, leavePotential);

                //Continue Resolving (Avoid Segment, Avoid Crossing, ...)
            }
        }

    }
}

void BacktrackingAvoidResolution::avoid(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    if(_current.predecessor_ == NULL)
        return;

    //We need to find if _next is a successor or a predecessor of current and use this crossing for avoidance
    bool crossingFound = false;
    std::vector<std::reference_wrapper<Vertex>> crossing;

    if(_current.crossingPredecessor)
    {
        crossing = _current.getPlanningPredecessors();

        for(const Vertex & v : crossing)
        {
            if(v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }

    if(!crossingFound && _current.crossingSuccessor)
    {
        crossing = _current.getPlanningSuccessors();

        for(const Vertex & v : crossing)
        {
            if(v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }


    if(crossingFound)
    {
        for(const Vertex & waitSeg : crossing)
        {
            if(waitSeg.getSegment().getSegmentId() != _next.getSegment().getSegmentId())
            {
                int32_t collision = -1;
                bool vertexFree = route_querry_->checkSegment(waitSeg, _current.predecessor_->potential + pCalc_->CalculatePotential(_current) - timeoverlap_, std::max<float>(_freePotential, _current.potential + pCalc_->CalculatePotential(waitSeg)) + timeoverlap_, robotDiameter_, collision);

                if(vertexFree || collision != -1)
                {
                    //Copy the current Vertex and set smallest possible potential (everithing else stays the same)
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
                    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    current_n.potential = current_n.predecessor_->potential + pCalc_->CalculatePotential(current_n);

                    //Copy the wait vertex insert it in the path and assign the right potential
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                    Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    wait_seg_n.potential = std::max<float>(_freePotential, current_n.potential + pCalc_->CalculatePotential(wait_seg_n));           //Set the potential as max of the next segment is free, and the minimum Vertex potential the robot can achive
                    wait_seg_n.collision = -1;
                    wait_seg_n.isWaitSegment = true;

                    wait_seg_n.successor_ = &_next;
                    wait_seg_n.predecessor_ = &current_n;

                    current_n.successor_ = &wait_seg_n;
                    //Reset _next to be ready for expansion (should be allready done in trackback, but safety first :D)
                    _next.potential = -1;
                    _next.collision = -1;

                    if(vertexFree)
                    {
                        foundSolutions_.push_back(wait_seg_n);
                    }
                    else if(collision != -1)
                    {
                        if(collision != _collision)
                            addCollision(collision);

                        float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_seg_n.getSegment().getSegmentId());
                        moveSegment(current_n, wait_seg_n, collision, leavePotential);
                    }
                }
            }
        }
    }
}

void BacktrackingAvoidResolution::moveSegment(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //We need to find the neighborhood where _current is not present
    std::vector<std::reference_wrapper<Vertex>> crossing;
    crossing = _next.getPlanningPredecessors();

    for(const Vertex & v : crossing)
    {
        if(v.getSegment().getSegmentId() == _current.getSegment().getSegmentId())
        {
            crossing = _next.getPlanningSuccessors();
            break;
        }
    }

    int collision = -1;
    //Check if we have enough time to move through the segment
    if(route_querry_->checkSegment(_next, _current.potential - timeoverlap_,  _current.potential + pCalc_->CalculatePotential(_next) + timeoverlap_, robotDiameter_, collision))
    {
        for(const Vertex & waitSeg : crossing)
        {
            float moveFwdTime = _current.potential + pCalc_->CalculatePotential(_next) + timeoverlap_;
            float waitTime = std::max<float>(moveFwdTime + pCalc_->CalculatePotential(waitSeg) + timeoverlap_, _freePotential + 2 * timeoverlap_);
            bool vertexIsFree = route_querry_->checkSegment(waitSeg, moveFwdTime - timeoverlap_, waitTime, robotDiameter_, collision);

            if(vertexIsFree || collision != -1)
            {
                //We need one Vertex for moving forward (next), one for waiting (waitSeg) and one for moving backward (next).
                //(We have to copy all three Vertex because every Vertex in the crossing is considered (backtracking...))

                //Copy Vertex used for moving away
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
                Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_fwd_n.potential = moveFwdTime;
                move_fwd_n.isWaitSegment = false;
                move_fwd_n.collision = -1;

                //Copy the Vertex for waiting
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                wait_n.potential = waitTime;
                wait_n.isWaitSegment = true;
                wait_n.collision = -1;


                //Copy Vertex used for moving back
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
                Vertex &move_bwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_bwd_n.potential = -1;
                move_bwd_n.isWaitSegment = false;
                move_bwd_n.collision = -1;

                move_fwd_n.successor_ = &wait_n;
                //pred allready set (curr)

                wait_n.successor_ = &move_bwd_n;
                wait_n.predecessor_ = &move_fwd_n;

                move_bwd_n.predecessor_ = &wait_n;
                move_bwd_n.successor_ = _next.successor_;


                if(vertexIsFree)
                {
                    foundSolutions_.push_back(wait_n);
                }
                else if(collision != -1)
                {
                    if(collision != _collision)
                        addCollision(collision);

                    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
                    moveSegment(move_fwd_n, wait_n, robotDiameter_, leavePotential);
                }
            }

        }
    }
    else if(collision != -1)
    {
        if(collision != _collision)
            addCollision(collision);

    }

}

/*
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
}*/
