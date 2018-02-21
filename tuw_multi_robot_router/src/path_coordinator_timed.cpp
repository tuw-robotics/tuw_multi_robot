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

#include <tuw_global_planner/path_coordinator_timed.h>

#define TIME_INFINITY   (-1)
//#define DEBUG

Path_Coordinator_Timed::Path_Coordinator_Timed() : Path_Coordinator()
{
}


bool Path_Coordinator_Timed::addPath(std::vector< std::shared_ptr< Segment > >& _path, int radius_pixel)
{
    for(int i = 0; i < _path.size(); i++)
    {
        int begin = 0;
        int end = -1;

        if(i != 0)
        {
            begin = _path[i - 1]->planning.Potential;
        }

        if(i != _path.size() - 1)
        {
            end = _path[i]->planning.Potential;
        }


         if(!timeline_.addSegment(begin, end, _path[i], activeRobot_, radius_pixel, true))
         {
             return false;
         }

        Neighbours pred = _path[i]->getPredecessors();

        if(pred.size() > 1)
        {
            for(auto it = pred.cbegin(); it != pred.cend(); it++)
            {
                if(!timeline_.addCrossingSegment(begin, end, (*it), activeRobot_, radius_pixel, false))
                {
                    return false;
                }

            }
        }

        Neighbours succ = _path[i]->getSuccessors();

        if(succ.size() > 1)
        {
            for(auto it = succ.cbegin(); it != succ.cend(); it++)
            {
                if(!timeline_.addCrossingSegment(begin, end, (*it), activeRobot_, radius_pixel, false))
                {
                    return false;
                }

            }
        }

    }

    return true;
}

bool Path_Coordinator_Timed::isGoal(std::shared_ptr< Segment > _seg)
{
    return _seg->getIndex() == goalSegments_[activeRobot_]->getIndex();
}

std::shared_ptr< Segment > Path_Coordinator_Timed::getEnd()
{
    return goalSegments_[activeRobot_];
}

std::shared_ptr< Segment > Path_Coordinator_Timed::getStart()
{
    return startSegments_[activeRobot_];
}





bool Path_Coordinator_Timed::checkSegment(std::shared_ptr<Segment> _next, int _startTime, int _endTime, int _radiusPixel, int& _collisionRobot, bool _ignoreGoal)
{
    if(!checkSegmentSingle(_next, _startTime, _endTime, _radiusPixel, _collisionRobot, _ignoreGoal))
    {
        return false;
    }

    Neighbours pred = _next->getPredecessors();

    if(pred.size() > 1)
    {
        for(auto it = pred.cbegin(); it != pred.cend(); it++)
        {
            if(!timeline_.checkCrossingSegment(_startTime, _endTime, (*it), activeRobot_, _radiusPixel, _collisionRobot))
            {
                return false;
            }

        }
    }

    Neighbours succ = _next->getSuccessors();

    if(succ.size() > 1)
    {
        for(auto it = succ.cbegin(); it != succ.cend(); it++)
        {
            if(!timeline_.checkCrossingSegment(_startTime, _endTime, (*it), activeRobot_, _radiusPixel, _collisionRobot))
            {
                return false;
            }

        }
    }


    return true;
}

void Path_Coordinator_Timed::updateNrOfCollisions(int _collisionRobot, int _collisions)
{
    robotCollisions_[activeRobot_][_collisionRobot] = _collisions;
}


bool Path_Coordinator_Timed::checkSegmentSingle(std::shared_ptr< Segment > _next, int _startTime, int _endTime, int _radiusPixel, int& _collisionRobot, bool _ignoreGoal)
{
    if(isGoal(_next) && !_ignoreGoal)
    {
        return timeline_.checkSegment(_startTime, TIME_INFINITY, _next, activeRobot_, _radiusPixel, _collisionRobot);
    }
    else
    {
        return timeline_.checkSegment(_startTime, _endTime, _next, activeRobot_, _radiusPixel, _collisionRobot);
    }
}


void Path_Coordinator_Timed::reset(std::vector< std::shared_ptr< Segment > > _graph, int _nrRobots)
{
    //timeline_ = new Timeline();
  
    timeline_.reset(_graph);
    robotCollisions_.resize(_nrRobots);

    for(auto & robotNr : robotCollisions_)
    {
        robotNr.clear();
        robotNr.resize(_nrRobots, 0);
    }
    
    for(std::shared_ptr<Segment> & seg : goalSegments_)
    {
        seg->clear();
    }
    
    for(std::shared_ptr<Segment> & seg : startSegments_)
    {
        seg->clear();
    }
    
    goalSegments_.clear();
    startSegments_.clear();
}

void Path_Coordinator_Timed::setActive(int _robotNr)
{
    activeRobot_ = _robotNr;
}

bool Path_Coordinator_Timed::setGoalSegments(const std::vector< std::shared_ptr< Segment > > _goalSegments, const std::vector< int > radius)
{
    for(std::shared_ptr<Segment> & seg : goalSegments_)
    {
        seg->clear();
    }
    
    
    goalSegments_.clear();
    
    goalSegments_ = _goalSegments;
    return true;
}

bool Path_Coordinator_Timed::setStartSegments(const std::vector< std::shared_ptr< Segment > > _startSegments, const std::vector< int > radius)
{
    for(std::shared_ptr<Segment> & seg : startSegments_)
    {
        seg->clear();
    }
    startSegments_.clear();
    
    startSegments_ = _startSegments;
    return true;
}



int Path_Coordinator_Timed::findSegNr(int _robot, int _timestep)
{
    int segId = timeline_.findSegId(_robot, _timestep);

    return segId;
}


int Path_Coordinator_Timed::findPotentialUntilRobotOnSegment(int _robot, std::shared_ptr< Segment > _segment)
{
    return timeline_.getTimeUntilRobotOnSegment(_robot, _segment);
}


std::vector< std::pair<int, float> > Path_Coordinator_Timed::getListOfRobotsHigherPrioritizedRobots(int _robot, std::shared_ptr<Segment> _segment)
{
    return timeline_.getListOfRobotsHigherPrioritizedRobots(_robot, _segment);
}



const std::vector< int >& Path_Coordinator_Timed::getNrOfRobotCollisions(int _robot)
{
    return robotCollisions_[_robot];
}








































Path_Coordinator_Timed::Timeline::Timeline()
{
}

void Path_Coordinator_Timed::Timeline::reset(std::vector< std::shared_ptr<Segment> > _graph)
{
    timeline_.clear();
    segmentSpace_.clear();
    maxTime_ = 0;

    for(int i = 0; i < _graph.size(); i++)
    {
        timeline_.emplace_back();
        segmentSpace_.push_back(_graph[i]->getPathSpace());
    }
}

bool Path_Coordinator_Timed::Timeline::addSegment(int _startTime, int _endTime, std::shared_ptr< Segment > _seg, int _robotNr, int _robotSize, bool _mainSeg)
{
    int collision;

    if(!checkSegment(_startTime, _endTime, _seg, _robotNr, _robotSize, collision))
    {
        return false;
    }

    int segmentIdx = _seg->getIndex();
    int freeSpace = segmentSpace_[segmentIdx];

    if(_endTime > maxTime_)
    {
        maxTime_ = _endTime;
    }

    //Segment has enough space
    if(robotSegments_.size() <= _robotNr)
    {
        robotSegments_.resize(_robotNr + 1);
    }

    robotSegments_[_robotNr].push_back(_seg->getIndex());
    timeline_[_seg->getIndex()].emplace_back(_robotNr, (float) _robotSize, _startTime, _endTime, _mainSeg);

    return true;
}

bool Path_Coordinator_Timed::Timeline::addCrossingSegment(int _startTime, int _endTime, std::shared_ptr< Segment > _seg, int _robotNr, int _robotSize, bool _mainSeg)
{
    if(_robotSize > _seg->getPathSpace())
    {
        return addSegment(_startTime, _endTime, _seg, _robotNr, _seg->getPathSpace(), _mainSeg);
    }

    return addSegment(_startTime, _endTime, _seg, _robotNr, _robotSize, _mainSeg);
}

bool Path_Coordinator_Timed::Timeline::checkCrossingSegment(int _startTime, int _endTime, std::shared_ptr< Segment > _seg, int _robotNr, int _robotSize, int& _lastCollisionRobot)
{
    if(_robotSize > _seg->getPathSpace())
    {
        return checkSegment(_startTime, _endTime, _seg, _robotNr, _seg->getPathSpace(), _lastCollisionRobot);
    }

    return checkSegment(_startTime, _endTime, _seg, _robotNr, _robotSize, _lastCollisionRobot);

}



bool Path_Coordinator_Timed::Timeline::checkSegment(int _startTime, int _endTime, std::shared_ptr< Segment > _seg, int _robotNr, int _robotSize, int& _lastCollisionRobot)
{
    _lastCollisionRobot = -1;
    int segmentIdx = _seg->getIndex();

    //Return if to less space
    int freeSpace = segmentSpace_[segmentIdx];

    if(freeSpace < _robotSize)
    {
        return false;
    }

    for(const seg_occupation & occupation : timeline_[segmentIdx])
    {
        if(occupation.robot != _robotNr)
        {
            if(_endTime == TIME_INFINITY || occupation.endTime == TIME_INFINITY)
            {
                if(_endTime == TIME_INFINITY && occupation.endTime == TIME_INFINITY)
                {
                    freeSpace -= occupation.spaceOccupied;
                    _lastCollisionRobot = occupation.robot;

                    if(freeSpace < _robotSize)
                    {
                        return false;
                    }
                }
                else if(_endTime == TIME_INFINITY && _startTime <= occupation.endTime)
                {
                    freeSpace -= occupation.spaceOccupied;
                    _lastCollisionRobot = occupation.robot;

                    if(freeSpace < _robotSize)
                    {
                        return false;
                    }
                }
                else if(occupation.endTime == TIME_INFINITY && _endTime >= occupation.startTime)
                {
                    freeSpace -= occupation.spaceOccupied;
                    _lastCollisionRobot = occupation.robot;

                    if(freeSpace < _robotSize)
                    {
                        return false;
                    }
                }
            }
            else
            {

                if((_startTime <= occupation.startTime && _endTime >= occupation.startTime) || (_startTime <= occupation.endTime && _endTime >= occupation.endTime))
                {
                    freeSpace -= occupation.spaceOccupied;
                    _lastCollisionRobot = occupation.robot;

                    if(freeSpace < _robotSize)
                    {
                        return false;
                    }
                }
                else if((_startTime >= occupation.startTime && _endTime <= occupation.endTime))
                {
                    freeSpace -= occupation.spaceOccupied;
                    _lastCollisionRobot = occupation.robot;

                    if(freeSpace < _robotSize)
                    {
                        return false;
                    }
                }
            }
        }
    }

    //No collision
    _lastCollisionRobot = -1;
    return true;
}

int Path_Coordinator_Timed::Timeline::getSize()
{
    return maxTime_;
}


int Path_Coordinator_Timed::Timeline::findSegId(int _robot, int _timestep)
{
    if(_robot < 0)
    {
        return -1;
    }

    for(const int & s : robotSegments_[_robot])
    {
        for(const seg_occupation & occupation : timeline_[s])
        {
            if(occupation.robot == _robot && _timestep >= occupation.startTime && _timestep <= occupation.endTime && occupation.mainSeg)
            {
                return s;
            }
        }
    }

    return -1;
}

int Path_Coordinator_Timed::Timeline::getTimeUntilRobotOnSegment(int _robotNr, std::shared_ptr< Segment > _seg)
{
    int segmentIndex = _seg->getIndex();

    int ret = -2;

    for(const auto & occupation : timeline_[segmentIndex])
    {
        if(occupation.robot == _robotNr)
        {
            ret = occupation.endTime;
        }
    }

    return ret;
}

std::vector< std::pair<int, float> > Path_Coordinator_Timed::Timeline::getListOfRobotsHigherPrioritizedRobots(int _robot, std::shared_ptr< Segment > _segment)
{
    std::vector<std::pair<int, float>> robots;

    seg_occupation occupationRobot(-1, -1, -1, -1);

    for(const auto & occupation : timeline_[_segment->getIndex()])
    {
        if(occupation.robot == _robot)
        {
            occupationRobot = occupation;
            break;
        }
    }

    if(occupationRobot.robot != _robot)
        return robots;    //TODO Error Should not happen


    for(const auto & occupation : timeline_[_segment->getIndex()])
    {
        if(occupation.robot != _robot && occupation.startTime < _segment->planning.Potential)
        {
            if(_segment->getPathSpace() - occupation.spaceOccupied < occupationRobot.spaceOccupied)
            {
                //Sort only if they would block each other
                robots.push_back( {occupation.robot, occupation.endTime});
            }
        }
    }

    return robots;
}

