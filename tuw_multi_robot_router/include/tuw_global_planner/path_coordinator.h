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

#ifndef PATH_COORDINATOR_H
#define PATH_COORDINATOR_H

#include <tuw_global_planner/utils.h>
#include <memory>
#include <vector>
#include <tuw_global_planner/segment.h>
#include <grid_map_ros/grid_map_ros.hpp>

class Path_Coordinator
{
public:
  virtual void reset(std::vector< std::shared_ptr<Segment> > _graph, int _nrRobots)=0;
  virtual bool addPath(std::vector<std::shared_ptr<Segment>> &_path, int radius_pixel)=0;
  virtual bool checkSegment(std::shared_ptr<Segment> _next, int _startTime, int _endTime, int _radius_pixel, int &_collisionRobot, bool ignoreGoal=false)=0;  
  virtual void setActive(int _robotNr)=0;
  virtual bool setGoalSegments(const std::vector<std::shared_ptr<Segment>> _goalSegments, const std::vector<int> radius)=0;
  virtual bool setStartSegments(const std::vector<std::shared_ptr<Segment>> _startSegments, const std::vector<int> radius)=0;
  virtual bool isGoal(std::shared_ptr<Segment> _seg)=0;
  virtual std::shared_ptr<Segment>  getStart()=0;
  virtual std::shared_ptr<Segment>  getEnd()=0;
  virtual std::vector<std::pair<int,float>> getListOfRobotsHigherPrioritizedRobots(int _robot, std::shared_ptr<Segment>  _segment)=0;
  virtual const std::vector<int> &getNrOfRobotCollisions(int _robot)=0;
  virtual void updateNrOfCollisions(int _collisionRobot,int _collisions)=0;
  
  
  virtual int findSegNr(int _robot, int _potential)=0;  //TODO BETTER VERSION
  virtual int findPotentialUntilRobotOnSegment(int _robot, std::shared_ptr< Segment > _segment)=0;
};

#endif // PATH_QUERRY_H
